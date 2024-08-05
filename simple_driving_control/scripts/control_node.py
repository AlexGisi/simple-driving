#!/usr/bin/python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

from simple_driving_control.vehicle_state import VehicleState
from simple_driving_control.controller import VehicleController
from simple_driving_control.parameters import VehicleParams, ControllerParams
from simple_driving_control.util import quat_to_yaw


class ControlNode:
    def __init__(self) -> None:
        rospy.init_node('control_node')

        self.steer_cmd_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        self.brake_cmd_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)
        self.throttle_cmd_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.marker_pub = rospy.Publisher('/target', Marker, queue_size=10)

        v_params = VehicleParams(wheelbase=2.65)
        c_params = ControllerParams(accel_k_d=4, accel_k_p=4)

        self.state = VehicleState(x=0, y=0, theta=0, v=0)
        self.controller = VehicleController(
            self.state,
            controller_params=c_params,
            vehicle_params=v_params,
        )

        self.target = np.array([20, 0])

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quat_to_yaw(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        v = np.linalg.norm([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        ])
        self.state.set(np.array([x, y, theta, v]))

    def send_cmd(self):
        cmd = self.controller.commands(self.target)
        if (throttle := cmd[0]) > 0:
            self.throttle_cmd_pub.publish(throttle)
        else:
            self.brake_cmd_pub.publish(-throttle)
        
        self.steer_cmd_pub.publish(cmd[1])

    def pub_target(self):
        m = Marker()
        m.header.frame_id = "world"
        m.type = 2  # Sphere
        m.pose.position.x = self.target[0]
        m.pose.position.y = self.target[1]
        self.marker_pub.publish(m)

    def run(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            self.send_cmd()
            self.pub_target()
            rate.sleep()


if __name__ == '__main__':
    node = ControlNode()
    node.run()
