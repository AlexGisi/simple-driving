#!/usr/bin/python3
import os
import time
import csv
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from simple_driving_control.vehicle_state import VehicleState
from simple_driving_control.util import quat_to_yaw


class LogNode:
    def __init__(self) -> None:
        rospy.init_node("log_node")

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.steer_cmd_pub = rospy.Subscriber("/steering_cmd", Float64, self.steer_cb)
        self.brake_cmd_pub = rospy.Subscriber("/brake_cmd", Float64, self.steer_cb)
        self.throttle_cmd_pub = rospy.Subscriber("/throttle_cmd", Float64, self.throttle_cb)
        self.target_pub = rospy.Subscriber('/target', Marker, self.target_cb)

        self.state = VehicleState(x=0, y=0, theta=0, v=0)
        self.throttle = 0
        self.steer = 0
        self.brake = 0
        self.target = (0, 0)

        self.log_fp = os.path.join(
            rospkg.RosPack().get_path("simple_driving_control"),
            "log",
            rospy.get_param("log_file_name") + ".csv",
        )

        if os.path.exists(self.log_fp):
            self.log_fp = os.path.join(
                rospkg.RosPack().get_path("simple_driving_control"),
                "log",
                rospy.get_param("log_file_name"),
                "-repeat.csv",
            )
            rospy.logwarn(f"log file exists, renamed to {self.log_fp}")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quat_to_yaw(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        v = np.linalg.norm(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
            ]
        )
        self.state.set(np.array([x, y, theta, v]))

    def throttle_cb(self, msg):
        self.throttle = msg.data

    def steer_cb(self, msg):
        self.steer = msg.data

    def target_cb(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y)

    def log(self):
        step_data = {
            "time": time.time(),
            "x": self.state.get_x(),
            "y": self.state.get_y(),
            "theta": self.state.get_theta(),
            "v": self.state.get_v(),
            "throttle": self.throttle,
            "steer": self.steer,
        }
        write_header = not os.path.exists(self.log_fp)
        with open(self.log_fp, mode="a+", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(step_data.keys()))
            if write_header:
                writer.writeheader()
            writer.writerow(step_data)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.log()
            rate.sleep()


if __name__ == "__main__":
    node = LogNode()
    node.run()
