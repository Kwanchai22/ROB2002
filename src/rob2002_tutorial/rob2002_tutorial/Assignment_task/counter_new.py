import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseArray
import cv2
import os

class Counter3D(Node):
    detection_threshold = 1.0 # in meters 

    def __init__(self):      
        super().__init__('counter_3d')
        self.total_objects = 6
        self.detected_objects = [] # list of all detected objects
        self.run = False
        self.screenshot_directory = "./screenshots" # Directory to save screenshots
        os.makedirs(self.screenshot_directory, exist_ok=True)

        # subscribe to object detector
        self.subscriber = self.create_subscription(PoseStamped, '/object_location', 
                                                   self.counter_callback,
                                                   qos_profile=qos.qos_profile_sensor_data)
        
        # publish all detected object as an array of poses
        self.publisher = self.create_publisher(PoseArray, '/object_count_array',
                                                      qos.qos_profile_parameters)

    def save_screenshot(self, image, bbox, object_id):
        """Save a screenshot of the detected object."""
        x, y, w, h = bbox
        cropped_image = image[y:y+h, x:x+w]
        filepath = os.path.join(self.screenshot_directory, f"object_{object_id}.png")
        cv2.imwrite(filepath, cropped_image)
        print(f"Saved screenshot for object {object_id} at {filepath}")

    def counter_callback(self, data):
        new_object = data.pose
        # check if the new object is away from all detected objects so far

        object_exists = False
        for object in self.detected_objects:
            # calculate the distance between the new_object and each in the list
            pos_a = object.position
            pos_b = new_object.position
            d = math.sqrt((pos_a.x - pos_b.x) ** 2 + (pos_a.y - pos_b.y) ** 2 + (pos_a.z - pos_b.z) ** 2)
            if d < self.detection_threshold: # found a close neighbour in the already existing list, so this one won't be added
                object_exists = True
                break

        if not object_exists: # new object!
            self.detected_objects.append(new_object)
            # Capture a screenshot for the new object (placeholder bbox values here)
            image = cv2.imread("placeholder_color_image.png") # Load or access the image frame
            bbox = (50, 50, 100, 100) # Replace with actual bounding box coordinates
            self.save_screenshot(image, bbox, len(self.detected_objects))

        # publish a PoseArray of object poses for visualisation in rviz
        parray = PoseArray(header=Header(frame_id=data.header.frame_id))
        for object in self.detected_objects:
            parray.poses.append(object)
        self.publisher.publish(parray)            

        # print to the console
        print(f'total count {len(self.detected_objects)}')
        if self.total_objects <= len(self.detected_objects):
            print("all objects detected")
            self.run = True
            sys.exit(0)
        for object in self.detected_objects:
            print(object.position)

def main(args=None):
    rclpy.init(args=args)
    counter_3d = Counter3D()
    
    # Run the node until `self.run` becomes True
    try:
        while rclpy.ok():
            rclpy.spin_once(counter_3d)
            if counter_3d.run:
                print("detected all objects.")
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        counter_3d.destroy_node()
        rclpy.shutdown()
        print("ROS 2 application terminated.")


if __name__ == '__main__':
    main()
