#!/usr/bin/env python3
"""Один прогон исследователя без захвата: страж стоит, ездит только исследователь.

Запускать в контейнере docker-hsl-adapter-1 (rclpy, sim time). Пишет JSON:
сколько проехал, самые длинные простои при ненулевой команде, статусы
планировщика/MPPI, контакты и минимальное расстояние до стража.

    python3 stuck_probe.py --seconds 120 --goal 0.5 3.5 > out.json
"""
import argparse
import json
import math
from collections import Counter

import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from hsl_interfaces.msg import PlanningIntent


class Probe(Node):
    def __init__(self, args):
        super().__init__("stuck_probe", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.args = args
        self.t0 = None
        self.samples = []            # (t, x, y, cmd_v, cmd_w)
        self.cmd = (0.0, 0.0)
        self.status = Counter()
        self.global_status = Counter()
        self.contacts = []           # (t, other)
        self.opponent = None
        self.min_opp = float("inf")
        self.create_subscription(Odometry, "navigation/self", self.on_self, 10)
        self.create_subscription(Odometry, "opponent/odom", self.on_opp, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.behavior = -1
        self.plan = {}
        self.mppi = {}
        self.diag = []
        self.last_diag = -1.0
        self.create_subscription(String, "navigation/planning_diagnostics", lambda m: setattr(self, "plan", self.parse(m.data)), 10)
        self.create_subscription(String, "navigation/mppi_diagnostics", lambda m: setattr(self, "mppi", self.parse(m.data)), 10)
        self.gstatus = ""
        self.pstatus = ""
        self.create_subscription(PlanningIntent, "navigation/intent", lambda m: setattr(self, "behavior", int(m.behavior)), 10)
        self.create_subscription(String, "navigation/planner_status", self.on_pstatus, 10)
        self.create_subscription(String, "navigation/global_status", self.on_gstatus, 10)
        self.create_subscription(ContactsState, "body_contacts", self.on_contact, 10)
        self.done = False

    @staticmethod
    def parse(text):
        try:
            return json.loads(text)
        except ValueError:
            return {}

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pstatus(self, m):
        self.pstatus = m.data
        self.status.update([m.data])

    def on_gstatus(self, m):
        self.gstatus = m.data
        self.global_status.update([m.data])

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_opp(self, m):
        self.opponent = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_contact(self, m):
        for s in m.states:
            self.contacts.append((self.now() - (self.t0 or 0), s.collision1_name.split("::")[0]))

    def on_self(self, m):
        t = self.now()
        if self.t0 is None:
            self.t0 = t
        p = m.pose.pose.position
        self.samples.append((t - self.t0, p.x, p.y, *self.cmd, self.behavior, self.gstatus, self.pstatus))
        if self.args.diag_out and t - self.last_diag >= 0.5:
            self.last_diag = t
            self.diag.append({"t": round(t - self.t0, 2), "pos": [round(p.x, 3), round(p.y, 3)],
                              "cmd": [round(self.cmd[0], 3), round(self.cmd[1], 3)], "behavior": self.behavior,
                              "plan": self.plan, "mppi": self.mppi, "opp": self.opponent})
        if self.opponent:
            self.min_opp = min(self.min_opp, math.hypot(p.x - self.opponent[0], p.y - self.opponent[1]))
        if (t - self.t0 >= self.args.seconds or
                math.hypot(p.x - self.args.goal[0], p.y - self.args.goal[1]) < 0.12):
            self.done = True


def report(node, args):
    s = node.samples
    out = {"samples": len(s)}
    if len(s) < 2:
        return out
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(s, s[1:]))
    # простой: окно, где за window_s смещение < 0.05 м, а команда ненулевая
    stalls, i = [], 0
    window = args.window
    while i < len(s):
        j = i
        while j < len(s) and s[j][0] - s[i][0] < window:
            j += 1
        if j >= len(s):
            break
        moved = math.hypot(s[j][1] - s[i][1], s[j][2] - s[i][2])
        commanded = sum(1 for k in range(i, j) if abs(s[k][3]) > 0.02 or abs(s[k][4]) > 0.1) / max(1, j - i)
        if moved < 0.05 and commanded > 0.5:
            k = j
            while k < len(s) and math.hypot(s[k][1] - s[i][1], s[k][2] - s[i][2]) < 0.05:
                k += 1
            seg = s[i:min(k, len(s) - 1) + 1]
            stalls.append({"t": round(s[i][0], 1), "dur": round(s[min(k, len(s) - 1)][0] - s[i][0], 1),
                           "at": [round(s[i][1], 2), round(s[i][2], 2)],
                           "behavior": dict(Counter(x[5] for x in seg)),
                           "global": dict(Counter(x[6] for x in seg)),
                           "local": dict(Counter(x[7] for x in seg)),
                           "cmd_v": round(sum(abs(x[3]) for x in seg) / len(seg), 3),
                           "cmd_w": round(sum(abs(x[4]) for x in seg) / len(seg), 3)})
            i = k
        else:
            i += 1
    last = s[-1]
    out.update({
        "duration_s": round(last[0], 1),
        "distance_m": round(dist, 2),
        "mean_speed": round(dist / max(last[0], 1e-6), 3),
        "final": [round(last[1], 2), round(last[2], 2)],
        "goal_dist": round(math.hypot(last[1] - args.goal[0], last[2] - args.goal[1]), 2),
        "reached": math.hypot(last[1] - args.goal[0], last[2] - args.goal[1]) < 0.12,
        "stalls": stalls,                       # простои при ненулевой команде
        "longest_stall_s": max([x["dur"] for x in stalls], default=0.0),
        "stall_total_s": round(sum(x["dur"] for x in stalls), 1),
        "behavior_time": dict(Counter(x[5] for x in s)),
        "contacts": dict(Counter(n for _, n in node.contacts)),
        "min_opponent_dist": None if node.min_opp == float("inf") else round(node.min_opp, 2),
        "planner_status": dict(node.status),
        "global_status": dict(node.global_status),
    })
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--goal", type=float, nargs=2, default=(0.5, 3.5))
    parser.add_argument("--window", type=float, default=6.0)
    parser.add_argument("--diag-out", default="")
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.2)
    if args.diag_out:
        with open(args.diag_out, "w") as f:
            json.dump(node.diag, f)
    print(json.dumps(report(node, args)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
