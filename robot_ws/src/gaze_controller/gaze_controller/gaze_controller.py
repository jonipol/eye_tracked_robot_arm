import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

from gaze_controller.bounding_box import BoundingBox

import math
import cv2
import numpy as np


class GazeController(Node):
    def __init__(self):
        super().__init__('gaze_controller')
        best_effort_qos = QoSProfile(
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        )
        self.gaze_sub = self.create_subscription(PointStamped,
                                                 '/gaze',
                                                 self.gaze_callback,
                                                 best_effort_qos)
        self.fiducial_sub = self.create_subscription(AprilTagDetectionArray,
                                                     '/tag_detections',
                                                     self.fiducial_callback,
                                                     best_effort_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo,
                                                        '/camera_info',
                                                        self.camera_info_callback,
                                                        best_effort_qos)

        self.latest_fiducial_msg: AprilTagDetectionArray = None
        self.camera_info = None

        self.marker_pose_0 = Point(x=0.5,  y=0.5,  z=0.0)
        self.marker_pose_1 = Point(x=0.5,  y=-0.5, z=0.0)
        self.marker_pose_2 = Point(x=-0.5, y=-0.5, z=0.0)
        self.marker_pose_3 = Point(x=-0.5, y=0.5,  z=0.0)
        self.marker_pose_list = [self.marker_pose_0, self.marker_pose_1, self.marker_pose_2, self.marker_pose_3]

    def gaze_callback(self, point: PointStamped):
        # Check timestamps

        # Check if gaze is inside area

        # solvePnP

        # Get gaze point in world

        # Move arm
        pass

    def fiducial_callback(self, tag_array: AprilTagDetectionArray):
        self.latest_fiducial_msg = tag_array

    def camera_info_callback(self, camera_info: CameraInfo):
        self.camera_info = camera_info
        self.camera_info_sub.destroy()      # TODO: Validate data before destroying.Also there is another way to destroy
        # TODO: Do not destroy if transient local -> it's fine to have the sub on all the time


if __name__ == "__main__":
    rclpy.init()
    gaze_controller = GazeController()
    gaze_controller.get_logger().info('Gaze controller node started')
    try:
        rclpy.spin(gaze_controller)
    except KeyboardInterrupt:
        pass
    finally:
        gaze_controller.destroy_node()
        gaze_controller.get_logger().info('Gaze controller node shutting down')
        rclpy.shutdown()
