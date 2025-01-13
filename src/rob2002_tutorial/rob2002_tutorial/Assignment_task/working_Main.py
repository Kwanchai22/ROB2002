import rclpy
from rclpy.node import Node
from rclpy import qos
import time 
import cv2 as cv
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Polygon, PolygonStamped, Point32
from cv_bridge import CvBridge
import sys

from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
# from .detector_dblcounting import DetectorBasic

from counter_3d import Counter3D
from detector_3d import Detector3D

class wander(Node):
    """
    A very simple Roamer implementation for LIMO.
    It goes straight until any obstacle is within
    a specified distance and then just turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        #On construction of the object, create a Subscriber
        #to listen to lasr scans and a Publisher to control
        #the robot
        """
        super().__init__('mover_laser')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        # self.data_ranges = self.create_publisher(Int32 , "/total_ranges", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)

        self.angular_range = 10
    
    def laserscan_callback(self, data):
        """
        #Callback called any time a new laser scan become available
        """
        total_ranges = len(data.ranges) ; 

        # self.get_logger().info(f'Total ranges:  {total_ranges}')
        # total_ranges_msg = Int32()
        # total_ranges_msg.data = total_ranges
        # self.publisher_total_ranges.publish(total_ranges_msg)

        left_range = data.ranges[:total_ranges //3]
        front_range = data.ranges[:total_ranges //3 : 2 * total_ranges //3]
        right_range = data.ranges[2*total_ranges //3:]

        min_dist_left = min(left_range)
        min_dist_front = min(front_range)
        min_dist_right = min (right_range)

        min_dist = min(data.ranges[int(len(data.ranges)/2) - self.angular_range : int(len(data.ranges)/2) + self.angular_range])
        # print(f'Min distance: {min_dist:.2f}')  # print the distance turning threshold
        # self.get_logger().info(f'Front: {min_dist_front:.2f} , Left: {min_dist_left:.2f} , Right: {min_dist_right:.2f}')
        msg = Twist()
        if min_dist< 1.0:
            if(min_dist_left >= min_dist_right):
             msg.linear.x = 0.0
             msg.angular.z = -0.5
             time.sleep(1)
            #  self.get_logger().info("Turning left")
            elif (min_dist_left <= min_dist_right):
                msg.linear.x = 0.0
                msg.angular.z = 0.5
                time.sleep(1)
                # self.get_logger().info("Turning right")
            else:
                 msg.linear.x = 0.0
        else:
            # self.get_logger().info("Moving forward")
            msg.linear.x = 0.2
        # self.publisher.publish(msg , total_ranges)
        self.publisher.publish(msg )


def main(args=None):
    rclpy.init(args=args)

    # Create instances of the nodes
    #mover = wander()
    detector_3d = Detector3D()
    counter_3d = Counter3D()

    # Use a MultiThreadedExecutor to spin the nodes
    executor = MultiThreadedExecutor()
    executor.add_node(detector_3d)
    executor.add_node(counter_3d)
    #executor.add_node(mover)

    try:
        # Continuously spin the executor until counter_3d.run becomes True
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # Allow other tasks to run
            if counter_3d.run:  # Check if the condition to exit is met
                print("All objects detected. Shutting down...")
                break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Clean up resources
        detector_3d.destroy_node()
        counter_3d.destroy_node()
        #mover.destroy_node()
        rclpy.shutdown()
        print("Program terminated.")
if __name__ == '__main__':
    main()