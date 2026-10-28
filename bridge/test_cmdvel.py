#!/usr/bin/env python3
"""验证 bridge 的 ROS 出口: /cmd_vel 发得出去, 也收得回来。

这个脚本只关心"命令能不能通过 DDS 落地", 不涉及雷达/点云。三种用法:

  1) 离线自测(没有小车也能跑, 但要有 ROS 环境):
         wheeltec_humble.sh python3 ~/ctrlscene/bridge/test_cmdvel.py
     自己发、自己收, 验证 geometry_msgs/Twist 的编解码和 DDS 通路都正常。
     这是 `lidar_bridge.py --source ros2` 那条分支能否驱动底盘的前提。

  2) 接上小车后, 看底盘到底有没有订阅 /cmd_vel:
         ./wheeltec_humble.sh python3 ~/ctrlscene/bridge/test_cmdvel.py --watch
     打印"订阅者数量"和收到(或没收到)的消息。

  3) 让小车真的动一下 —— 务必先把车架空, 轮子离地:
         ./wheeltec_humble.sh python3 ~/ctrlscene/bridge/test_cmdvel.py \
             --drive 0.05 0.0 --seconds 3
     只发很小的线速度, 3 秒后自动补零速度停下。

注意: 跑之前先确认没有别的 bridge / wheeltec-control 在发 /cmd_vel,
否则两边会打架, 看日志分不清是谁发的。
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser(
        description='/cmd_vel 通路探针', formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument('--topic', default='/cmd_vel')
    ap.add_argument('--seconds', type=float, default=2.0, help='收/发持续多久')
    ap.add_argument('--rate', type=float, default=20.0, help='发令频率 Hz')
    ap.add_argument('--watch', action='store_true', help='只监听, 不自己发令')
    ap.add_argument('--drive', nargs=2, type=float, metavar=('LINEAR', 'ANGULAR'),
                    help='真的发这个速度(先架空!)')
    args = ap.parse_args()

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
    except ImportError as exc:
        raise SystemExit(
            '找不到 rclpy / geometry_msgs。请在 Humble 容器里运行:\n'
            '    ~/桌面/ROS/wheeltec_humble.sh python3 <本文件>\n'
            f'(原始错误: {exc})')

    rclpy.init()
    node = Node('cmdvel_probe')
    pub = node.create_publisher(Twist, args.topic, 10)
    received = []

    def on_msg(msg):
        received.append((round(msg.linear.x, 3), round(msg.angular.z, 3)))

    node.create_subscription(Twist, args.topic, on_msg, 10)

    # 先转一会儿让 DDS 完成发现, 否则订阅者数量会是 0
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    subs = pub.get_subscription_count()
    print(f'话题 {args.topic}  订阅者 {subs} 个'
          f'{"  (含本脚本自己的订阅, 所以至少是 1)" if subs else ""}')

    if args.watch:
        print(f'监听 {args.seconds:.1f}s ...')
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if received:
            print(f'收到 {len(received)} 条, 最后一条 线速度/角速度 = {received[-1]}')
        else:
            print('一条都没收到。检查: 域号是否一致 (ROS_DOMAIN_ID)、有没有发布者')
        node.destroy_node()
        rclpy.shutdown()
        return 0

    if args.drive:
        linear, angular = args.drive
    else:
        linear, angular = 0.0, 0.0

    print(f'发布 {args.rate:.0f}Hz 共 {args.seconds:.1f}s: '
          f'线速度 {linear:+.3f} m/s, 角速度 {angular:+.3f} rad/s'
          + ('   [--drive, 车会动!]' if args.drive else '   [零速度, 安全]'))

    period = 1.0 / max(1.0, args.rate)
    sent = 0
    end = time.monotonic() + args.seconds
    last = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now - last >= period:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            pub.publish(msg)
            sent += 1
            last = now
        rclpy.spin_once(node, timeout_sec=0.005)

    # 收尾: 无论刚才发什么, 最后一定要把零速度落到
    for _ in range(5):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
    print(f'发完 {sent} 帧, 已补 5 帧零速度')

    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    if not received:
        print('✗ 自己发的消息自己都没收到 —— DDS 通路有问题'
              '(域号不一致? /cmd_vel 被别的类型占用?)')
        return 1
    nonzero = [r for r in received if r != (0.0, 0.0)]
    print(f'✓ 回环收到 {len(received)} 条, 其中非零 {len(nonzero)} 条, '
          f'样本 {nonzero[0] if nonzero else received[0]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
