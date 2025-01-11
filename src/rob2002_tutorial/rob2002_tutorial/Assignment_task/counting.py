import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32
import math

class ObjectCounter(Node):
    def __init__(self):
        super().__init__('object_counter')

        self.object_sub = self.create_subscription(PolygonStamped, '/object_polygon', 
                                                   self.object_callback, 10)
        
        self.tracked_objects = []  # List to store detected objects (coordinates)
        self.distance_threshold = 50.0  # Minimum distance to consider objects as different
        self.object_count = 0  # Total number of unique objects counted

        self.get_logger().info("Object Counter Node started")

    def object_callback(self, msg):
        new_object = self.extract_coordinates(msg.polygon)

        if not self.is_duplicate(new_object):
            self.tracked_objects.append(new_object)
            self.object_count += 1
            self.get_logger().info(f"New object detected! Total count: {self.object_count}")
        else:
            self.get_logger().info("Duplicate object detected. Ignoring.")

    def extract_coordinates(self, polygon):
        """Extracts the average coordinates of a polygon."""
        x_coords = [point.x for point in polygon.points]
        y_coords = [point.y for point in polygon.points]
        avg_x = sum(x_coords) / len(x_coords)
        avg_y = sum(y_coords) / len(y_coords)
        return (avg_x, avg_y)

    def is_duplicate(self, new_object):
        """Checks if the object is within the threshold distance of any tracked object."""
        for tracked in self.tracked_objects:
            if self.euclidean_distance(new_object, tracked) < self.distance_threshold:
                return True
        return False

    def euclidean_distance(self, obj1, obj2):
        """Calculates the Euclidean distance between two objects."""
        return math.sqrt((obj1[0] - obj2[0]) ** 2 + (obj1[1] - obj2[1]) ** 2)

def main(args=None):
    rclpy.init(args=args)
    object_counter = ObjectCounter()

    try:
        rclpy.spin(object_counter)
    except KeyboardInterrupt:
        object_counter.get_logger().info("Shutting down Object Counter Node")
    finally:
        object_counter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
