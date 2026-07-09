#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import yaml


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detection')

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

        self.declare_parameter('yaml_path', '~/.ros/camera_info/puzz_cam.yaml')
        yaml_path = os.path.expanduser(
            self.get_parameter('yaml_path').get_parameter_value().string_value)


        self.declare_parameter('cube_id', 17)
        self.cube_id = self.get_parameter('cube_id').get_parameter_value().integer_value


        self.declare_parameter('marker_length',      0.08)
        self.declare_parameter('cube_marker_length', 0.045)
        marker_len = self.get_parameter('marker_length').value
        cube_len   = self.get_parameter('cube_marker_length').value


        self.declare_parameter('camera_offset_x', 0.1241)
        self.dx = self.get_parameter('camera_offset_x').value

        self.mtx = None
        self.dist = None
        self.zero_dist = np.zeros((5, 1), dtype=np.float32)
        self.camera_info_received = False

        self.load_camera_info(yaml_path)


        self.obj_points_wall = self._square_points(marker_len)
        self.obj_points_cube = self._square_points(cube_len)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.image_callback,
            10)

        self.pub_detections = self.create_publisher(
            Float32MultiArray, '/aruco_detections', 10)

        self.pub_target = self.create_publisher(
            Float32MultiArray, '/aruco_target', 10)

        self.get_logger().info(
            f'ArUco detector | cube_id={self.cube_id} | '
            f'L_pared={marker_len} m L_cubo={cube_len} m | dx={self.dx} m')

    @staticmethod
    def _square_points(length):
        h = length / 2.0
        return np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0]], dtype=np.float32)

    def load_camera_info(self, yaml_path):
        if not os.path.exists(yaml_path):
            self.get_logger().error(
                f"No se encontró archivo de calibración en {yaml_path}. "
                f"Usando valores genéricos.")
            self.mtx = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                                dtype=np.float32)
            self.dist = np.array([0, 0, 0, 0, 0], dtype=np.float32)
            self.camera_info_received = True
            return

        try:
            with open(yaml_path, 'r') as file:
                calib_data = yaml.safe_load(file)
            self.mtx = np.array(calib_data["camera_matrix"]["data"],
                                dtype=np.float32).reshape((3, 3))
            self.dist = np.array(calib_data["distortion_coefficients"]["data"],
                                 dtype=np.float32)
            self.camera_info_received = True

        except Exception as e:
            self.get_logger().error(f"Error procesando YAML: {e}")

    def image_callback(self, msg):
        if not self.camera_info_received:
            return

        try:
            frame_raw = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error decodificando imagen: {e}")
            return

        frame = cv2.undistort(frame_raw, self.mtx, self.dist, None, self.mtx)

        corners, ids, rejected = self.detector.detectMarkers(frame)

        detections = Float32MultiArray()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                marker_id  = int(ids[i][0])
                obj_points = (self.obj_points_cube if marker_id == self.cube_id
                              else self.obj_points_wall)
                corners_2d = corners[i].reshape((4, 2))

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corners_2d, self.mtx, self.zero_dist,
                    flags=cv2.SOLVEPNP_ITERATIVE)
                if not success:
                    continue

                cv2.drawFrameAxes(frame, self.mtx, self.zero_dist,
                                  rvec, tvec, 0.05)

                x_cam = tvec[0][0]
                z_cam = tvec[2][0]

                x_robot = z_cam + self.dx
                y_robot = -x_cam

                r   = np.sqrt(x_robot**2 + y_robot**2)
                phi = np.arctan2(y_robot, x_robot)

                if marker_id == self.cube_id:

                    R, _ = cv2.Rodrigues(rvec)
                    nz = R[:, 2]
                    yaw_m = np.arctan2(nz[0], nz[2]) - np.pi
                    yaw_m = np.arctan2(np.sin(yaw_m), np.cos(yaw_m))

                    msg_t = Float32MultiArray()

                    msg_t.data = [float(y_robot), float(r),
                                  float(yaw_m), float(phi)]
                    self.pub_target.publish(msg_t)
                else:
                    detections.data.extend(
                        [float(marker_id), float(r), float(phi)])

        if detections.data:
            self.pub_detections.publish(detections)



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
