#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler


# the same route but expressed as x, y and angle as partial quarterion (z, w)
waypoint_route_quat2 = [
    [1.58, -2.69, 0.7, 0.8],
    [-0.6, -2.82, -0.97, 0.21],
    [-2.3, -1.78, 0.8, 0.58],
    [-1.64, 0.95, 0.83, 0.55],
    [1.31, 0.7, 0.24, 0.97],
    [3.2, 2.52, 0.98, 0.21],
    [0.0, 0.0, 0.0, 0.0],
]

waypoint_route_quat = [
    [-0.15, -2.76, -0.76, 0.65],
    [2.62, -3.34, 0.68, 0.74],
    [-1.67, -2.07, -1.0, 0.01],
    [-1.99, -0.5, 0.59, 0.81],
    [0.78, 0.27, 0.6, 0.8],
    [0.98, 2.86, 0.98, 0.19],
    [-1.87, 3.18, -0.99, 0.15],
    [-1.73, 2.8, -0.62, 0.78],
    [0.93, 0.69, -0.17, 0.99],
    [3.42, 0.11, 0.62, 0.78],
    [3.78, 3.03, 0.97, 0.23],
    

    
]

def pose_from_xyquat(timestamp, x=0.0, y=0.0, pz=0.0, pw=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = timestamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = pz
    pose.pose.orientation.w = pw
    return pose

def pose_from_xytheta(timestamp, x=0.0, y=0.0, theta=0.0):
    # negative theta: turn clockwise
    q = quaternion_from_euler(0, 0, theta)
    return pose_from_xyquat(timestamp, x, y, q[2], q[3])


def main():

    rclpy.init()

    # publish the current waypoint location (for visualisation in rviz)
    publisher = Node("waypoint_publisher").create_publisher(PoseStamped, "/current_waypoint", qos_profile=qos.qos_profile_parameters)

    navigator = BasicNavigator()

    # Set the initial pose
    initial_pose = pose_from_xyquat(navigator.get_clock().now().to_msg())

    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Prepare a set of waypoints 
    waypoints = []
    # if you would rather specify the orientation as quaternion use the following line instead
    for wp in waypoint_route_quat:
         waypoints.append(pose_from_xyquat(navigator.get_clock().now().to_msg(), *wp))

    # follow the specified waypoints
    navigator.followWaypoints(waypoints)

    # Some feedback on the progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        # publish currrent waypoint
        publisher.publish(waypoints[feedback.current_waypoint])        
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(waypoints))
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Task complete!')
    elif result == TaskResult.CANCELED:
        print('Task was canceled.')
    elif result == TaskResult.FAILED:
        print('Task failed!')


if __name__ == '__main__':
    main()
