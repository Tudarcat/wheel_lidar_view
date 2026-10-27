#!/usr/bin/env python3
"""WHEELTEC 视觉桥 —— ROS2/离线数据 → Godot 的低延迟 UDP 中间件。

Godot 不能直接连 ROS2, 所以由这个脚本把传感器数据打成紧凑的二进制包, 通过
UDP 发给同机的 Godot。设计上分成 "数据源(source)" 和 "出口(sink)" 两层,
后续加相机、里程计、电池只增加消息类型, 不用改传输层。

    数据源                      出口
    ┌──────────────┐
    │ ros2  /scan  │──┐
    │ bag   回放   │──┼──→ 打包 → UDP 127.0.0.1:9100 → Godot
    │ demo  合成   │──┘
    └──────────────┘

用法:
    # 实车(需要在 ROS 环境里跑, 比如 wheeltec_humble.sh 里面)
    ./bridge/lidar_bridge.py --source ros2

    # 回放之前录的 bag(不需要 ROS!)
    ./bridge/lidar_bridge.py --source bag --bag bags/wheeltec_demo

    # 合成的房间扫描, 用来调界面
    ./bridge/lidar_bridge.py --source demo

协议见 PROTOCOL.md
"""
import argparse
import json
import math
import os
import socket
import sqlite3
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# ---------------------------------------------------------------------------
# 协议常量
# ---------------------------------------------------------------------------
MAGIC = b'WLDR'
VERSION = 1

MSG_SCAN = 1      # 激光扫描
MSG_STATUS = 2    # 状态(JSON)
MSG_IMAGE = 3     # 预留: 相机 JPEG
MSG_TELEMETRY = 4 # 底盘遥测(里程计 + IMU + 电压)

# magic ver type  n  seq  sec  nsec  amin  ainc  rmin  rmax  flags rsvd
HEADER = struct.Struct('<4sBBHIIIffffHH')
assert HEADER.size == 40, HEADER.size

FLAG_RANGE_MM = 0x0001   # 负载是 u16 毫米, 0 表示无效


@dataclass
class Scan:
    seq: int
    stamp_sec: int
    stamp_nsec: int
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: list


def encode_scan(scan: Scan) -> bytes:
    """把扫描打成一包。距离换算成 u16 毫米, 省一半带宽还能塞进一个 MTU。"""
    rmin, rmax = scan.range_min, scan.range_max
    values = []
    for r in scan.ranges:
        if not math.isfinite(r) or r < rmin or r > rmax:
            values.append(0)
        else:
            values.append(min(65535, max(1, int(round(r * 1000.0)))))
    head = HEADER.pack(MAGIC, VERSION, MSG_SCAN, len(values), scan.seq,
                       scan.stamp_sec, scan.stamp_nsec,
                       scan.angle_min, scan.angle_increment, rmin, rmax,
                       FLAG_RANGE_MM, 0)
    return head + struct.pack('<%dH' % len(values), *values)


@dataclass
class Telemetry:
    """底盘状态。字段全部是可选的, 用 flags 标出哪些有效。"""
    pos_x: float = 0.0
    pos_y: float = 0.0
    yaw: float = 0.0
    vel_x: float = 0.0
    vel_y: float = 0.0
    omega: float = 0.0
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    voltage: float = 0.0
    flags: int = 0


TELEMETRY = struct.Struct('<10fHH')   # 10 个 f32 + 2 个 u16
assert TELEMETRY.size == 44, TELEMETRY.size

TEL_ODOM = 0x0001
TEL_IMU = 0x0002
TEL_BATTERY = 0x0004


def encode_telemetry(tel: Telemetry) -> bytes:
    head = HEADER.pack(MAGIC, VERSION, MSG_TELEMETRY, 0, 0, 0, 0,
                       0.0, 0.0, 0.0, 0.0, 0, 0)
    return head + TELEMETRY.pack(
        tel.pos_x, tel.pos_y, tel.yaw,
        tel.vel_x, tel.vel_y, tel.omega,
        tel.acc_x, tel.acc_y, tel.acc_z,
        tel.voltage, tel.flags, 0)


def encode_status(info: dict) -> bytes:
    body = json.dumps(info, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    head = HEADER.pack(MAGIC, VERSION, MSG_STATUS, 0, 0, 0, 0,
                       0.0, 0.0, 0.0, 0.0, 0, 0)
    return head + body


# ---------------------------------------------------------------------------
# 数据源
# ---------------------------------------------------------------------------
class Source:
    name = 'base'

    def read(self):
        """返回下一帧 Scan, 没有数据时返回 None。内部自己处理节奏。"""
        raise NotImplementedError

    def telemetry(self):
        """返回最新的底盘状态 Telemetry, 没有就返回 None。"""
        return None

    def describe(self):
        return self.name

    def close(self):
        pass


class DemoSource(Source):
    """合成一个矩形房间 + 一个缓慢移动的障碍物, 用来在没有车时调界面。"""
    name = 'demo'

    def __init__(self, rate=10.0, points=540):
        self.period = 1.0 / rate
        self.n = points
        self.seq = 0
        self.t0 = time.monotonic()

    def read(self):
        time.sleep(self.period)
        now = time.monotonic()
        t = now - self.t0
        angle_min = -math.pi
        inc = 2 * math.pi / self.n
        ranges = []
        box_a = math.pi / 3 + t * 0.6          # 缓慢绕行的方块
        for i in range(self.n):
            a = angle_min + i * inc
            # 房间: 8m x 6m 的墙
            dx, dy = math.cos(a), math.sin(a)
            tx = abs(4.0 / dx) if abs(dx) > 1e-6 else 1e9
            ty = abs(3.0 / dy) if abs(dy) > 1e-6 else 1e9
            r = min(tx, ty)
            # 障碍物: 一个 2.5m 外、约 8 度宽的点簇
            if abs(((a - box_a + math.pi) % (2 * math.pi)) - math.pi) < 0.07:
                r = min(r, 2.5)
            # 制造几条无效点, 模拟真实激光的漏回波
            if i % 211 == 0:
                ranges.append(float('inf'))
            else:
                ranges.append(round(r + (i % 5) * 0.002, 4))
        self.seq += 1
        self._last_t = t
        return Scan(self.seq, int(now), int((now % 1) * 1e9),
                    angle_min, inc, 0.15, 50.0, ranges)

    def telemetry(self):
        """合成一条缓慢变化的速度/电压, 用来在没有车时调底盘面板。"""
        t = time.monotonic() - self.t0
        speed = 0.10 + 0.06 * math.sin(t * 0.4)
        omega = 0.22 * math.sin(t * 0.9)
        return Telemetry(
            pos_x=1.6 * math.sin(t * 0.12),
            pos_y=1.1 * math.cos(t * 0.09),
            yaw=math.atan2(math.cos(t * 0.09), math.sin(t * 0.12)),
            vel_x=speed, vel_y=0.0, omega=omega,
            acc_x=0.06 * 0.4 * math.cos(t * 0.4),
            acc_y=0.0,
            acc_z=9.78 + 0.02 * math.sin(t * 3.0),
            voltage=11.2 - 0.25 * abs(math.sin(t * 0.05)),
            flags=TEL_ODOM | TEL_IMU | TEL_BATTERY)


class RosSource(Source):
    """订阅 ROS2 的 sensor_msgs/LaserScan。必须在有 ROS 环境的终端里运行。"""

    def __init__(self, topic, domain_id, odom_topic='/odom', imu_topic='/imu/data_raw',
                 battery_topic='/PowerVoltage', timeout=5.0):
        os.environ.setdefault('ROS_DOMAIN_ID', str(domain_id))
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, LaserScan
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('wheeltec_godot_bridge')
        self.latest = None
        self.received = 0
        self.seq = 0
        self.tel = Telemetry()
        self.node.create_subscription(LaserScan, topic, self._callback, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.node.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.node.create_subscription(Float32, battery_topic, self._on_battery, 10)
        self.name = f'ros2:{topic}'
        print(f'等待 {topic} 的第一帧 ...', file=sys.stderr)

    def _callback(self, msg):
        self.latest = msg
        self.received += 1

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.tel.pos_x = msg.pose.pose.position.x
        self.tel.pos_y = msg.pose.pose.position.y
        self.tel.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.tel.vel_x = msg.twist.twist.linear.x
        self.tel.vel_y = msg.twist.twist.linear.y
        self.tel.omega = msg.twist.twist.angular.z
        self.tel.flags |= TEL_ODOM

    def _on_imu(self, msg):
        self.tel.acc_x = msg.linear_acceleration.x
        self.tel.acc_y = msg.linear_acceleration.y
        self.tel.acc_z = msg.linear_acceleration.z
        self.tel.flags |= TEL_IMU

    def _on_battery(self, msg):
        self.tel.voltage = msg.data
        self.tel.flags |= TEL_BATTERY

    def telemetry(self):
        return self.tel if self.tel.flags else None

    def read(self):
        deadline = time.monotonic() + 0.5
        while self.latest is None and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.latest is None:
            return None
        msg, self.latest = self.latest, None
        self.received += 1
        self.seq += 1
        return Scan(seq=self.seq,
                    stamp_sec=msg.header.stamp.sec,
                    stamp_nsec=msg.header.stamp.nanosec,
                    angle_min=msg.angle_min,
                    angle_increment=msg.angle_increment,
                    range_min=msg.range_min,
                    range_max=msg.range_max,
                    ranges=list(msg.ranges))

    def close(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


# ---- 离线 bag 回放: 自带一个最小 CDR 解析器, 不需要装 ROS -------------------
class _CDR:
    def __init__(self, data):
        self.buf = data
        self.little = data[1] == 0x01
        self.endian = '<' if self.little else '>'
        self.offset = 4
        self.pos = 4

    def _align(self, size):
        rel = self.pos - self.offset
        rem = rel % size
        if rem:
            self.pos += size - rem

    def _num(self, fmt, size):
        self._align(size)
        value = struct.unpack_from(self.endian + fmt, self.buf, self.pos)[0]
        self.pos += size
        return value

    def int32(self):
        return self._num('i', 4)

    def uint32(self):
        return self._num('I', 4)

    def float32(self):
        return self._num('f', 4)

    def float64(self):
        return self._num('d', 8)

    def nums(self, count, fmt='d', size=8):
        self._align(size)
        values = list(struct.unpack_from(self.endian + '%d%s' % (count, fmt), self.buf, self.pos))
        self.pos += size * count
        return values

    def string(self):
        n = self.uint32()
        if n == 0:
            return ''
        raw = self.buf[self.pos:self.pos + n - 1]
        self.pos += n
        return raw.decode('utf-8', 'replace')


def decode_laserscan(data: bytes) -> Scan:
    c = _CDR(data)
    sec = c.int32()
    nanosec = c.uint32()
    c.string()                                  # frame_id
    angle_min = c.float32()
    angle_max = c.float32()                     # noqa: F841  (保留给调试)
    angle_increment = c.float32()
    c.float32()                                 # time_increment
    c.float32()                                 # scan_time
    range_min = c.float32()
    range_max = c.float32()
    count = c.uint32()
    c._align(4)
    ranges = list(struct.unpack_from(c.endian + '%df' % count, c.buf, c.pos))
    return Scan(seq=0, stamp_sec=sec, stamp_nsec=nanosec,
                angle_min=angle_min, angle_increment=angle_increment,
                range_min=range_min, range_max=range_max, ranges=ranges)


def _cdr_header(c):
    sec, nsec = c.int32(), c.uint32()
    c.string()          # frame_id
    return sec, nsec


def decode_odometry(data: bytes) -> dict:
    """nav_msgs/Odometry -> 位姿 + 速度(只取我们要的字段)"""
    c = _CDR(data)
    _cdr_header(c)
    c.string()                                  # child_frame_id
    px, py = c.float64(), c.float64()
    c.float64()                                 # z
    qx, qy, qz, qw = c.float64(), c.float64(), c.float64(), c.float64()
    c.nums(36)                                  # pose covariance
    vx, vy = c.float64(), c.float64()
    c.float64()                                 # linear z
    c.float64(), c.float64()                    # angular x, y
    wz = c.float64()
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return {'pos_x': px, 'pos_y': py, 'yaw': yaw,
            'vel_x': vx, 'vel_y': vy, 'omega': wz}


def decode_imu(data: bytes) -> dict:
    """sensor_msgs/Imu -> 线加速度"""
    c = _CDR(data)
    _cdr_header(c)
    c.nums(4)                                   # orientation
    c.nums(9)                                   # orientation covariance
    c.nums(3)                                   # angular velocity
    c.nums(9)
    ax, ay, az = c.float64(), c.float64(), c.float64()
    return {'acc_x': ax, 'acc_y': ay, 'acc_z': az}


def decode_float32(data: bytes) -> float:
    return _CDR(data).float32()


class BagSource(Source):
    """回放 rosbag2 的 .db3, 按包里记录的时间戳节奏发送。"""

    name = 'bag'

    def __init__(self, path, loop=False, speed=1.0, topic='/scan'):
        base = Path(path)
        if base.is_dir():
            dbs = sorted(base.glob('*.db3'))
            if not dbs:
                raise SystemExit(f'{path} 里没有 .db3 文件')
            db = dbs[0]
        else:
            db = base
        if not db.exists():
            raise SystemExit(f'找不到 bag: {db}')
        self.db = db
        self.loop = loop
        self.speed = max(0.05, speed)
        self.seq = 0
        self._load(topic)
        self._load_extra()
        self.index = 0
        self.prev_stamp = None
        self.tel = Telemetry()
        self.tel_state = {}

    def _load_extra(self):
        """可选地载入 /odom /imu/data_raw /PowerVoltage, 有就一起回放。"""
        self.extras = []
        for topic, decoder, key in (('/odom', decode_odometry, 'odom'),
                                    ('/imu/data_raw', decode_imu, 'imu'),
                                    ('/PowerVoltage', decode_float32, 'battery')):
            con = sqlite3.connect(str(self.db))
            cur = con.cursor()
            row = cur.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
            rows = []
            if row is not None:
                rows = cur.execute(
                    'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
                    (row[0],)).fetchall()
            con.close()
            if rows:
                self.extras.append({'topic': topic, 'decoder': decoder, 'key': key,
                                    'rows': rows, 'idx': 0})
                print(f'  遥测: {topic} {len(rows)} 条', file=sys.stderr)

    def telemetry(self):
        return self.tel if self.tel.flags else None

    def _load(self, topic):
        con = sqlite3.connect(str(self.db))
        cur = con.cursor()
        row = cur.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
        if row is None:
            names = [r[0] for r in cur.execute('SELECT name FROM topics')]
            con.close()
            raise SystemExit(f'bag 里没有 {topic}。可用话题: {", ".join(names)}')
        self.rows = cur.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
            (row[0],)).fetchall()
        con.close()
        if not self.rows:
            raise SystemExit(f'{topic} 在 bag 里没有消息')
        self.name = f'bag:{self.db.name}/{topic}'
        span = (self.rows[-1][0] - self.rows[0][0]) / 1e9
        print(f'已载入 {len(self.rows)} 帧, 时长 {span:.1f}s, 来自 {self.db.name}',
              file=sys.stderr)

    def read(self):
        if self.index >= len(self.rows):
            if not self.loop:
                return None
            self.index = 0
            self.prev_stamp = None
        stamp_ns, blob = self.rows[self.index]
        self.index += 1
        # 按原始时间戳的间隔来 pacing
        if self.prev_stamp is not None:
            gap = (stamp_ns - self.prev_stamp) / 1e9 / self.speed
            if gap > 0:
                time.sleep(min(gap, 1.0))
        self.prev_stamp = stamp_ns
        self._advance_telemetry(stamp_ns)
        scan = decode_laserscan(bytes(blob))
        self.seq += 1
        scan.seq = self.seq
        return scan

    def _advance_telemetry(self, stamp_ns):
        """把每个遥测话题的游标推进到 <= 当前扫描时间戳的最新一条。"""
        for extra in getattr(self, 'extras', []):
            rows = extra['rows']
            idx = extra['idx']
            while idx < len(rows) and rows[idx][0] <= stamp_ns:
                data = bytes(rows[idx][1])
                try:
                    value = extra['decoder'](data)
                except Exception:
                    value = None
                if value is not None:
                    if extra['key'] == 'battery':
                        self.tel.voltage = value
                        self.tel.flags |= TEL_BATTERY
                    else:
                        for k, v in value.items():
                            setattr(self.tel, k, v)
                        self.tel.flags |= TEL_ODOM if extra['key'] == 'odom' else TEL_IMU
                idx += 1
            extra['idx'] = idx

    def exhausted(self):
        return self.index >= len(self.rows)


def make_source(args):
    if args.source == 'ros2':
        return RosSource(args.topic, args.domain,
                         odom_topic=args.odom_topic,
                         imu_topic=args.imu_topic,
                         battery_topic=args.battery_topic)
    if args.source == 'demo':
        return DemoSource(rate=args.rate)
    if args.source == 'bag':
        if not args.bag:
            raise SystemExit('--source bag 需要 --bag 指向 bag 目录或 .db3')
        return BagSource(args.bag, loop=args.loop, speed=args.speed, topic=args.topic)
    raise SystemExit(f'未知 source: {args.source}')


# ---------------------------------------------------------------------------
# 出口
# ---------------------------------------------------------------------------
class UdpSink:
    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent = 0

    def send(self, payload: bytes):
        try:
            self.sock.sendto(payload, self.addr)
            self.sent += 1
        except OSError as exc:
            print(f'UDP 发送失败: {exc}', file=sys.stderr)

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description='ROS2/离线数据 → Godot 的 UDP 中间件',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('--source', choices=['ros2', 'bag', 'demo'], default='demo')
    ap.add_argument('--topic', default='/scan', help='ROS 话题, 默认 /scan')
    ap.add_argument('--bag', help='bag 目录或 .db3 路径')
    ap.add_argument('--loop', action='store_true', help='bag 循环回放')
    ap.add_argument('--speed', type=float, default=1.0, help='bag 回放倍速')
    ap.add_argument('--rate', type=float, default=10.0, help='demo 的扫描频率')
    ap.add_argument('--domain', type=int, default=42, help='ROS_DOMAIN_ID')
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--imu-topic', default='/imu/data_raw')
    ap.add_argument('--battery-topic', default='/PowerVoltage')
    ap.add_argument('--host', default='127.0.0.1', help='Godot 地址')
    ap.add_argument('--port', type=int, default=9100, help='Godot 端口')
    ap.add_argument('--quiet', action='store_true', help='不打印状态行')
    ap.add_argument('--dump', action='store_true', help='打印前几帧的解析结果')
    args = ap.parse_args()

    source = make_source(args)
    sink = UdpSink(args.host, args.port)
    print(f'数据源 {source.describe()}  →  udp {args.host}:{args.port}', file=sys.stderr)

    t0 = time.monotonic()
    frames = 0
    dropped = 0
    last_report = 0.0
    last_telemetry = 0.0
    dumped = 0
    try:
        while True:
            scan = source.read()
            if scan is None:
                if isinstance(source, BagSource) and source.exhausted():
                    break
                continue
            payload = encode_scan(scan)
            if len(payload) > 1472:
                # 超过 MTU 也只是本机回环, 但记一笔方便以后跨机部署时排查
                dropped += 1
            sink.send(payload)
            frames += 1

            # 底盘遥测按 5Hz 发: 它比激光慢得多, 没必要每帧都发
            now_tel = time.monotonic()
            if now_tel - last_telemetry >= 0.2:
                telemetry = source.telemetry()
                if telemetry is not None:
                    sink.send(encode_telemetry(telemetry))
                    last_telemetry = now_tel

            if args.dump and dumped < 3:
                dumped += 1
                valid = [r for r in scan.ranges if math.isfinite(r) and r > 0]
                print(f'[dump] seq={scan.seq} n={len(scan.ranges)} '
                      f'valid={len(valid)} min={min(valid, default=0):.2f} '
                      f'max={max(valid, default=0):.2f} '
                      f'amin={scan.angle_min:.3f} ainc={scan.angle_increment:.5f} '
                      f'包长={len(payload)}B')
                time.sleep(0.2)
            if args.dump and dumped == 3:
                break

            now = time.monotonic()
            if not args.quiet and now - last_report >= 1.0:
                last_report = now
                elapsed = now - t0
                hz = frames / elapsed if elapsed > 0 else 0
                info = {
                    'source': source.describe(),
                    'hz': round(hz, 2),
                    'frames': frames,
                    'points': len(scan.ranges),
                    'port': args.port,
                    'uptime': round(elapsed, 1),
                }
                sink.send(encode_status(info))
                print(f'\r  发送 {frames} 帧  {hz:5.1f} Hz  '
                      f'{len(scan.ranges)} 点  包长 {len(payload)}B '
                      f'{"" if not dropped else f"(超MTU {dropped})"}', end='', flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        source.close()
        sink.close()
    elapsed = time.monotonic() - t0
    print(f'\n结束: 共发送 {frames} 帧, 用时 {elapsed:.1f}s', file=sys.stderr)


if __name__ == '__main__':
    main()
