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
from counter_3d import Counter3D
from detector_3d import Detector3D
from detector_dblcounting import DetectorBasic


def main(args=None):
    rclpy.init(args=args)

    # Create instances of the nodes
    detector_3d = Detector3D()
    counter_3d = Counter3D()
    detector_dblcounting = DetectorBasic()

    # Use a MultiThreadedExecutor to spin the nodes
    executor = MultiThreadedExecutor()
    executor.add_node(detector_dblcounting)
    executor.add_node(detector_3d)
    executor.add_node(counter_3d)
    

    try:
        
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # Allow other tasks to run
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Clean up resources
        detector_3d.destroy_node()
        counter_3d.destroy_node()
        detector_dblcounting.destroy_node()
        rclpy.shutdown()
        print("Program terminated.")
if __name__ == '__main__':
    main()
