#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetPhysicsProperties
import tf

import numpy as np
import threading

XMIN = 0
YMIN = 1
ZMIN = 2
XMAX = 3
YMAX = 4
ZMAX = 5

class SafetyCheck:
    # map information
    boundary = None
    blocks = None
    frame_id = None
    
    # robot information
    radius = None
    
    rate = None
    
    # ROS objects
    tf_listener = None # tf listener to get the pose of the robot
    timer = None
    pause_service = None
    
    ##########
    def __init__(self):
        # Initialize map info using parameter server
        self.boundary = rospy.get_param('~boundary')
        self.blocks = rospy.get_param('~blocks')
        self.map_frame_id = rospy.get_param('~map_frame_id')
        
        # Initialize robot information
        self.radius = rospy.get_param('radius', 0.25)
        self.rate = rospy.get_param('rate', 100.)
        
        # Inflate blocks by robot radius
        for b in self.blocks:
            b[XMIN] = b[XMIN] - self.radius
            b[YMIN] = b[YMIN] - self.radius
            b[ZMIN] = b[ZMIN] - self.radius
            b[XMAX] = b[XMAX] + self.radius
            b[YMAX] = b[YMAX] + self.radius
            b[ZMAX] = b[ZMAX] + self.radius
        
        # Initialize ROS objects
        self.tf_listener = tf.TransformListener()
        
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.wait_for_service('/gazebo/get_physics_properties')
        try:
            self.pause_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            physics_service = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        props = physics_service()
        while props.pause:
            rospy.sleep(1.0)
            props = physics_service()
        
        rospy.sleep(5.0)
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)

    ##########
    def get_current_pose(self):
        position = None
        yaw = None
        try:
            # look up the current pose of the base_footprint using the tf tree
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame_id, '/ardrone/base_link', rospy.Time(0))
            # Convert to RRT state
            position = (trans[0], trans[1], trans[2])
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not get robot pose')
        
        return (position, yaw)
    
    ##########
    def timer_callback(self, event):
        # Get current pose and set as start pose
        (position, yaw) = self.get_current_pose()
        if position is None or yaw is None:
            return
        # Check for collisions
        if position[0] < self.boundary[XMIN] or position[0] > self.boundary[XMAX] \
           or position[1] < self.boundary[YMIN] or position[1] > self.boundary[YMAX] \
           or position[2] < self.boundary[ZMIN] or position[2] > self.boundary[ZMAX]:
            self.pause_service()
            rospy.logerr('Gazebo paused: The robot left the environment')
        for b in self.blocks:
            if position[0] >= b[XMIN] and position[0] <= b[XMAX] \
               and position[1] >= b[YMIN] and position[1] <= b[YMAX] \
               and position[2] >= b[ZMIN] and position[2] <= b[ZMAX]:
                self.pause_service()
                rospy.logerr('Gazebo paused: The robot collided with a block')
        
if __name__ == '__main__':
    rospy.init_node('safety_check_node')
    rrtp = SafetyCheck()
    rospy.spin()
    

