import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import CameraInfo, Image
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

from gaze_controller.bounding_box import BoundingBox
from point_in_polygon import is_inside_polygon

import math
import cv2
import numpy as np
from typing import List


class GazeController(Node):
    def __init__(self):
        super().__init__('gaze_controller')
        best_effort_qos = QoSProfile(
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        )

        transient_local_qos = QoSProfile(
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
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

        self.camera_sub = self.create_subscription(Image,
                                                   '/camera',
                                                   self.camera_callback,
                                                   best_effort_qos)

        self.latest_fiducial_msg: AprilTagDetectionArray = None
        self.camera_info = None

        self.marker_pose_0 = Point(x=0.5,  y=0.5,  z=0.0)
        self.marker_pose_1 = Point(x=0.5,  y=-0.5, z=0.0)
        self.marker_pose_2 = Point(x=-0.5, y=-0.5, z=0.0)
        self.marker_pose_3 = Point(x=-0.5, y=0.5,  z=0.0)
        self.marker_pose_list = [self.marker_pose_0, self.marker_pose_1, self.marker_pose_2, self.marker_pose_3]

    # TODO: combine to one callback using message filter.
    #       Check timestamps happens there?
    def gaze_callback(self, gaze_point: PointStamped):
        # get the area
        tag_detections: List[AprilTagDetectionArray] = []      # TODO: Get from message filter
        working_area_in_image: List[Point] = []
        for tag in tag_detections:
            working_area_in_image.append(tag.center)

        # Check which tags are visible -> mofidy marker_pose_list
        object_points = self.marker_pose_list

        # Check if gaze is inside area
        if is_inside_polygon(working_area_in_image, gaze_point):
            # solvePnP
            rotation_vector, translation_vector = cv2.solvePnP(object_points,
                                                               working_area_in_image,
                                                               cameraMatrix=0,
                                                               distCoeffs=0)
            # Get gaze point in world
            point = Point(x=1.5, y=1.5)

            # Move arm


    def camera_callback(self, data: Image):
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
