import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # For camera data
from geometry_msgs.msg import Point  # To store object position
import numpy as np
import cv2
from cv_bridge import CvBridge


class ObjectPositionTracker(Node):
    def __init__(self):
        super().__init__('object_position_tracker')
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.bridge = CvBridge()
        self.tracked_objects = []  # List to store tracked objects and their positions
        self.proximity_threshold = 0.1  # Distance threshold for considering objects as the same
        
        self.get_logger().info("Object Position Tracker Node Initialized")

    def camera_callback(self, msg):
        # Convert the image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Detect objects (using a placeholder function for object detection)
        detected_objects = self.detect_objects(frame)
        
        # Process each detected object
        for obj in detected_objects:
            position = self.estimate_position(obj)
            if not self.is_duplicate(position):
                self.tracked_objects.append(position)
                self.get_logger().info(f"New object detected at {position}")
            else:
                self.get_logger().info(f"Duplicate object ignored at {position}")
    
    def detect_objects(self, frame):
        """Placeholder for object detection logic. 
        Replace with a real object detection model."""
        # Example: Dummy detected objects' 2D bounding boxes
        detected_bounding_boxes = [[50, 50, 100, 100], [200, 200, 250, 250]]  # x_min, y_min, x_max, y_max
        return detected_bounding_boxes

    def estimate_position(self, bbox):
        """Estimate the object's position relative to the camera.
        Replace this with actual depth data or camera calibration logic."""
        # For simplicity, let's return a dummy position based on the center of the bounding box
        x_min, y_min, x_max, y_max = bbox
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        depth = 1.0  # Assume a fixed depth for this example
        return Point(x=center_x, y=center_y, z=depth)

    def is_duplicate(self, position):
        """Check if the position matches any tracked object within the proximity threshold."""
        for tracked_pos in self.tracked_objects:
            distance = np.linalg.norm([
                position.x - tracked_pos.x,
                position.y - tracked_pos.y,
                position.z - tracked_pos.z,
            ])
            if distance < self.proximity_threshold:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
