import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32, Point
import numpy as np


class ObjectPositionTracker(Node):
    def __init__(self):
        super().__init__('object_position_tracker')
        self.object_sub = self.create_subscription(PolygonStamped, '/object_polygon', self.object_callback, 10)
        self.tracked_objects = []  # List to store tracked object positions
        self.proximity_threshold = 0.1  # Distance threshold for considering objects as the same

    def object_callback(self, msg):
        # Extract the position from the PolygonStamped message
        position = self.extract_position(msg)

        # Check if it's a new object or a duplicate
        if not self.is_duplicate(position):
            self.tracked_objects.append(position)
            self.get_logger().info(f"New object detected at {position}")
            # Notify that an object has been found
            print("Found a new object!")
        else:
            self.get_logger().info(f"Duplicate object ignored at {position}")

    def extract_position(self, msg):
        """Extract the position from the PolygonStamped message."""
        # For simplicity, assume the position is the centroid of the bounding box
        polygon = msg.polygon
        x_min = polygon.points[0].x
        y_min = polygon.points[0].y
        x_max = polygon.points[1].x
        y_max = polygon.points[1].y
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        return Point(x=center_x, y=center_y, z=0.0)  # Assuming z=0 for 2D position

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
    object_tracker = ObjectPositionTracker()

    rclpy.spin(object_tracker)

    object_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
