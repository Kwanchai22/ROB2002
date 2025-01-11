# Python libraries
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import qos
import math

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge

# ROS Messages
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose


class Detector3D(Node):
    def __init__(self):
        super().__init__('detector3d')
        self.bridge = CvBridge()

        # Camera models and image data
        self.ccamera_model = None
        self.dcamera_model = None
        self.image_depth_ros = None
        self.color2depth_aspect = None

        # Parameters
        self.min_area_size = 100.0
        self.global_frame = 'odom'
        self.camera_frame = 'depth_link'
        self.visualisation = True
        self.real_robot = False

        # Camera topics
        ccamera_info_topic = '/limo/depth_camera_link/camera_info'
        dcamera_info_topic = '/limo/depth_camera_link/depth/camera_info'
        cimage_topic = '/limo/depth_camera_link/image_raw'
        dimage_topic = '/limo/depth_camera_link/depth/image_raw'

        if self.real_robot:
            ccamera_info_topic = '/camera/color/camera_info'
            dcamera_info_topic = '/camera/depth/camera_info'
            cimage_topic = '/camera/color/image_raw'
            dimage_topic = '/camera/depth/image_raw'
            self.camera_frame = 'camera_color_optical_frame'

        # Subscribers
        self.ccamera_info_sub = self.create_subscription(
            CameraInfo, ccamera_info_topic, self.ccamera_info_callback, qos.qos_profile_sensor_data)
        self.dcamera_info_sub = self.create_subscription(
            CameraInfo, dcamera_info_topic, self.dcamera_info_callback, qos.qos_profile_sensor_data)
        self.cimage_sub = self.create_subscription(
            Image, cimage_topic, self.image_color_callback, qos.qos_profile_sensor_data)
        self.dimage_sub = self.create_subscription(
            Image, dimage_topic, self.image_depth_callback, qos.qos_profile_sensor_data)

        # Publisher
        self.object_location_pub = self.create_publisher(
            PoseStamped, '/object_location', qos.qos_profile_parameters)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def color2depth_calc(self):
        if self.color2depth_aspect is None and self.ccamera_model and self.dcamera_model:
            self.color2depth_aspect = (math.atan2(self.ccamera_model.width, 2 * self.ccamera_model.fx()) / self.ccamera_model.width) \
                / (math.atan2(self.dcamera_model.width, 2 * self.dcamera_model.fx()) / self.dcamera_model.width)

    def image2camera_tf(self, image_coords, image_color, image_depth):
        # Transform from color to depth coordinates
        depth_coords = np.array(image_depth.shape[:2]) / 2 + (
            np.array(image_coords) - np.array(image_color.shape[:2]) / 2) * self.color2depth_aspect

        # Boundary check for depth coordinates
        depth_coords = np.clip(depth_coords, [0, 0], np.array(image_depth.shape[:2]) - 1)
        depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]

        # Project image coordinates to camera coordinates
        camera_coords = np.array(self.ccamera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])))
        camera_coords /= camera_coords[2]
        camera_coords *= depth_value

        # Return Pose in camera coordinates
        return Pose(position=Point(x=camera_coords[0], y=camera_coords[1], z=camera_coords[2]),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

    def ccamera_info_callback(self, data):
        if self.ccamera_model is None:
            self.ccamera_model = image_geometry.PinholeCameraModel()
            self.ccamera_model.fromCameraInfo(data)
            self.color2depth_calc()

    def dcamera_info_callback(self, data):
        if self.dcamera_model is None:
            self.dcamera_model = image_geometry.PinholeCameraModel()
            self.dcamera_model.fromCameraInfo(data)
            self.color2depth_calc()

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        if self.color2depth_aspect is None or self.image_depth_ros is None:
            return

        # Convert images to OpenCV format
        image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        if self.real_robot:
            image_depth /= 1000.0

        # Detect red objects in color image
        image_mask = cv2.inRange(image_color, (0, 0, 80), (50, 50, 255))
        object_contours, _ = cv2.findContours(image_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process contours
        for num, cnt in enumerate(object_contours):
            area = cv2.contourArea(cnt)
            if area > self.min_area_size:
                cmoms = cv2.moments(cnt)
                image_coords = (cmoms["m01"] / cmoms["m00"], cmoms["m10"] / cmoms["m00"])
                camera_pose = self.image2camera_tf(image_coords, image_color, image_depth)

                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.global_frame,
                        self.camera_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.time.Duration(seconds=1.0)
                    )
                    global_pose = do_transform_pose(PoseStamped(pose=camera_pose), transform)
                    self.object_location_pub.publish(global_pose)
                    self.get_logger().info(f"Object {num}: Global Position {global_pose.pose.position.x}, {global_pose.pose.position.y}, {global_pose.pose.position.z}")
                except LookupException:
                    self.get_logger().error(f"Transform not found: {self.camera_frame} to {self.global_frame}")
                except ExtrapolationException:
                    self.get_logger().error("Extrapolation failed")

                if self.visualisation:
                    cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 5, (255, 0, 0), -1)

        # Visualize
        if self.visualisation:
            cv2.imshow("Color Image", cv2.resize(image_color, None, fx=0.5, fy=0.5))
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detector = Detector3D()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

