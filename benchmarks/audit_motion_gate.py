#!/usr/bin/env python3
"""Read both final command streams while motion is forbidden, without issuing commands."""
import argparse
import json
import math
import time


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from rclpy.qos import QoSProfile, DurabilityPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=15)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('duel_gate_audit')
    observed = {prefix: {'allowed': None, 'commands': []} for prefix in ('', '/opponent')}
    subscriptions = []
    state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for prefix, data in observed.items():
        subscriptions.append(node.create_subscription(
            Bool, prefix + '/match/allowed',
            lambda msg, data=data: data.update(allowed=msg.data), state_qos))
        def command(msg, data=data):
            data['commands'].append([msg.linear.x, msg.linear.y, msg.angular.z])
        subscriptions.append(node.create_subscription(Twist, prefix + '/cmd_vel', command, 10))
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(data['allowed'] is not None and len(data['commands']) >= 10
                   for data in observed.values()):
                break
        result = {}
        for prefix, data in observed.items():
            pubs = node.get_publishers_info_by_topic(prefix + '/cmd_vel')
            zero = bool(data['commands']) and all(
                math.isfinite(value) and abs(value) < 1e-9
                for command in data['commands'] for value in command)
            result[prefix or '/'] = dict(data, publishers=[
                {'name': p.node_name, 'namespace': p.node_namespace} for p in pubs],
                passed=data['allowed'] is False and len(data['commands']) >= 10 and
                       zero and len(pubs) == 1)
        print(json.dumps(result, indent=2))
        return 0 if all(data['passed'] for data in result.values()) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
