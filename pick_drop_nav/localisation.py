#!/usr/bin/env python3

import math

import rclpy
import numpy as np
import tf_transformations
from numpy.linalg import inv
from rclpy import qos
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DeadReckoningClass(Node):
    def __init__(self):
        super().__init__('localisation')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

        self.declare_parameter("wheel_base", 0.19)     
        self.declare_parameter("wheel_radius", 0.05)   
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("q", 0.0)               
        self.declare_parameter("frame_prefix", "")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("meas_noise_r", 0.1)
        self.declare_parameter("meas_noise_phi", 0.1)

        self.L = self.get_parameter("wheel_base").value
        self.R = self.get_parameter("wheel_radius").value
        self.x = self.get_parameter("x").value
        self.y = self.get_parameter("y").value
        self.theta = self.get_parameter("q").value
        self.frame_prefix = self.get_parameter("frame_prefix").value
        self.rate = self.get_parameter("rate").value

        self.Ek = np.zeros((3, 3))
        self.R_meas = np.diag([
            self.get_parameter("meas_noise_r").value,
            self.get_parameter("meas_noise_phi").value,
        ])

        self.wR = 0.0
        self.wL = 0.0

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "world"
        self.odom_msg.child_frame_id = "odom"

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "world"
        self.tf_msg.child_frame_id = self.frame_prefix + "base_footprint"

        self.marker_map = {
            871: (-1.0, 0.0),
            811: (0.0, -1.3),
            9: (1.0, 0.0),
            8: (0.15, 1.3),
        }

        self.pub_odom = self.create_publisher(Odometry, "odom", 1)
        my_qos = qos.qos_profile_sensor_data
        self.create_subscription(Float32, "VelocityEncR", self.wR_cb, my_qos)
        self.create_subscription(Float32, "VelocityEncL", self.wL_cb, my_qos)
        
       
        self.create_subscription(Float32MultiArray, '/aruco_detections', self.aruco_cb, 10)

        self.create_timer(1.0 / self.rate, self.odometry_cb)
        self.t0 = self.get_clock().now()
        self.get_logger().info("Localisation node iniciado.")

    def wL_cb(self, msg):
        self.wL = msg.data

    def wR_cb(self, msg):
        self.wR = msg.data

    def aruco_cb(self, msg: Float32MultiArray):
        data = msg.data
        
        
        for i in range(0, len(data) - 2, 3):
            marker_id = int(data[i])
            

            if marker_id not in self.marker_map:
                continue


            r_meas = float(data[i + 1])
            phi_meas = float(data[i + 2])
            z_t = np.array([[r_meas], [phi_meas]]) 

            mx, my = self.marker_map[marker_id]


            delta_x = mx - self.x
            delta_y = my - self.y
            q = delta_x**2 + delta_y**2
            r_pred = math.sqrt(q)
            

            if r_pred < 1e-6:
                continue

            phi_pred = math.atan2(delta_y, delta_x) - self.theta
            phi_pred = math.atan2(math.sin(phi_pred), math.cos(phi_pred))
            h_x = np.array([[r_pred], [phi_pred]])

            H_t = np.array([
                [-delta_x / r_pred, -delta_y / r_pred, 0.0],
                [delta_y / q,       -delta_x / q,      -1.0],
            ])

            S = H_t @ self.Ek @ H_t.T + self.R_meas
            K_t = self.Ek @ H_t.T @ inv(S)

            residual = z_t - h_x
            residual[1, 0] = math.atan2(math.sin(residual[1, 0]), math.cos(residual[1, 0]))

            correction = K_t @ residual
            self.x += float(correction[0, 0])
            self.y += float(correction[1, 0])
            self.theta += float(correction[2, 0])
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            I = np.eye(3)
            IKH = I - K_t @ H_t
            self.Ek = IKH @ self.Ek @ IKH.T + K_t @ self.R_meas @ K_t.T 


    def odometry_cb(self):
        now = self.get_clock().now()
        if self.t0.nanoseconds == 0:
            self.t0 = now
            return

        dt = (now - self.t0).nanoseconds / 1e9
        if dt <= 0.0:
            self.t0 = now
            return
        self.t0 = now

        v = (self.wR + self.wL) * self.R / 2.0
        w = (self.wR - self.wL) * self.R / self.L


        sig2R = (0.0301 * abs(v)) + 0.0571
        sig2L = (0.0383 * abs(v)) + 0.0450
        M = np.array([[sig2R, 0.0], [0.0, sig2L]])

        sq = math.sin(self.theta)
        cq = math.cos(self.theta)


        Gx = np.array([
            [1.0, 0.0, -dt * v * sq],
            [0.0, 1.0,  dt * v * cq],
            [0.0, 0.0,  1.0],
        ])
        Gu = (0.5 * self.R * dt) * np.array([
            [cq, cq],
            [sq, sq],
            [2.0 / self.L, -2.0 / self.L],
        ])

        self.Ek = Gx @ self.Ek @ Gx.T + Gu @ M @ Gu.T

        self.x += dt * v * cq
        self.y += dt * v * sq
        self.theta += dt * w
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odometry(now)

    def publish_odometry(self, now):
        q_tf = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        self.odom_msg.header.stamp = now.to_msg()
        p = self.odom_msg.pose
        p.pose.position.x = self.x
        p.pose.position.y = self.y
        p.pose.position.z = 0.0
        p.pose.orientation.x = q_tf[0]
        p.pose.orientation.y = q_tf[1]
        p.pose.orientation.z = q_tf[2]
        p.pose.orientation.w = q_tf[3]

        p.covariance[0] = float(self.Ek[0, 0])
        p.covariance[1] = float(self.Ek[0, 1])
        p.covariance[5] = float(self.Ek[0, 2])
        p.covariance[6] = float(self.Ek[1, 0])
        p.covariance[7] = float(self.Ek[1, 1])
        p.covariance[11] = float(self.Ek[1, 2])
        p.covariance[14] = 1e-9
        p.covariance[21] = 1e-9
        p.covariance[28] = 1e-9
        p.covariance[30] = float(self.Ek[2, 0])
        p.covariance[31] = float(self.Ek[2, 1])
        p.covariance[35] = float(self.Ek[2, 2])
        self.pub_odom.publish(self.odom_msg)

        self.tf_msg.header.stamp = now.to_msg()
        t = self.tf_msg.transform
        t.translation.x = self.x
        t.translation.y = self.y
        t.translation.z = 0.0
        t.rotation.x = q_tf[0]
        t.rotation.y = q_tf[1]
        t.rotation.z = q_tf[2]
        t.rotation.w = q_tf[3]
        self.tf_broadcaster.sendTransform(self.tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningClass()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
