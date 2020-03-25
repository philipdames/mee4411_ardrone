#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

import numpy as np
import threading

from rrt.rrt import RRT
from rrt.rrt_star import RRTStar
from rrt.rrt_star_bid import RRTStarBidirectional
from rrt.rrt_star_bid_h import RRTStarBidirectionalHeuristic
from rrt.rrt_connect import RRTConnect
from search_space.search_space import SearchSpace

XMIN = 0
YMIN = 1
ZMIN = 2
XMAX = 3
YMAX = 4
ZMAX = 5

class RRTPlanner:
    # map information
    boundary = None
    blocks = None
    frame_id = None
    
    # robot information
    radius = None
    
    # RRT objects
    space = None # search space object
    rrt_type = None # type of RRT planner
    rrt = None # RRT planner object
    
    # RRT parameters
    length = None # length of edges to add to RRT [int]
    num_edges = None # number of edges to add at each iteration [int]
    Q = None # data structure used by RRT planners of form (length, num_edges)
    r = None # length of smallest edge to check for intersection with obstacles [float]
    max_samples = None # max number of samples to take before timing out [int]
    rewire_count = None # optional, number of nearby branches to rewire [int]
    prc = None # probability of checking for a connection to goal [float]
    
    # ROS objects
    goal_sub = None # subscriber to get the goal
    tf_listener = None # tf listener to get the pose of the robot
    path_pub = None # publisher to send the robot path
    
    ##########
    def __init__(self):
        # Initialize map information
        boundary = rospy.get_param('~boundary')
        blocks = rospy.get_param('~blocks')
        self.map_frame_id = rospy.get_param('~map_frame_id')
        
        # Initialize RRT parameters
        self.rrt_type = rospy.get_param('~rrt_type', 'rrt')
        self.length = rospy.get_param('rrt_length', 10)
        self.num_edges = rospy.get_param('rrt_num_edges', 4)
        self.Q = np.array([(self.length, self.num_edges)])  
        self.r = rospy.get_param('rrt_r', .1)
        self.max_samples = rospy.get_param('rrt_max_samples', 1024)
        self.rewire_count = rospy.get_param('rrt_rewire_count', 32)  
        self.prc = rospy.get_param('rrt_prc', 0.1)
        
        # Initialize robot information
        self.radius = rospy.get_param('radius', 0.25)
        
        # Initialize ROS objects
        self.goal_sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        self.tf_listener = tf.TransformListener()
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        
        ##### YOUR CODE STARTS HERE #####
        # dimensions of search space
        bounds = None # Must be of form np.array([(dim0 min, dim0 max), (dim1 min, dim1 max), ...])
        
        # dimensions of the obstacles (assumed to be axis aligned rectangles)
        obstacles = None # must be an array of the form [obstacle 0, obstacle 1, ...] where each obstacls is of the form (dim0 min, dim1 min,..., dim0 max, dim1 max)
        ##### YOUR CODE ENDS HERE #####
        
        # Initialize RRT search space
        self.space = SearchSpace(bounds, obstacles)
    
    ##########
    def get_current_pose(self):
        start = None
        try:
            # look up the current pose of the robot using the tf tree
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame_id, '/ardrone/base_link', rospy.Time(0))
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            
            # Convert to RRT state
            start = (trans[0], trans[1], trans[2], yaw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Could not get robot pose')
        
        return start
    
    ##########
    def rrt_to_ros_path(self, path_in):
        path_out = Path() # Initialize empty ROS path message
        ##### YOUR CODE STARTS HERE #####
        # Convert RRT path_in to ROS message path_out
        
        
        ##### YOUR CODE ENDS HERE #####
        
        return path_out
    
    ##########
    def goal_callback(self, msg):
        rospy.logdebug('RRTPlanner: Got goal')
        
        # Get current pose and set as start pose
        start = self.get_current_pose()
        if start is None:
            return
        
        ##### YOUR CODE STARTS HERE #####
        # Convert input msg to RRT goal of format (x, y, z, yaw)
        goal = None # goal location
        ##### YOUR CODE ENDS HERE #####

        # create RRT search object and find the path
        if self.rrt_type.lower() == 'rrt':
            rrt = RRT(self.space, self.Q, start, goal, self.max_samples, self.r, self.prc)
            path = rrt.rrt_search()
        elif self.rrt_type.lower() == 'rrtstar':
            rrt = RRTStar(self.space, self.Q, start, goal, self.max_samples, self.r, self.prc, self.rewire_count)
            path = rrt.rrt_star()
        elif self.rrt_type.lower() == 'rrt_star_bid':
            rrt = RRTStarBidirectional(self.space, self.Q, start, goal, self.max_samples, self.r, self.prc, self.rewire_count)
            path = rrt.rrt_star_bidirectional()
        elif self.rrt_type.lower() == 'rrt_star_bid_h':
            rrt = RRTStarBidirectionalHeuristic(self.space, self.Q, start, goal, self.max_samples, self.r, self.prc, self.rewire_count)
            path = rrt.rrt_star_bid_h()
        else:
            rospy.logerr('RRT type not defined')
        
        ##### YOUR CODE STARTS HERE #####
        # Check to make sure end of path is goal, only publish the path if it is
        
        
        # Convert to ROS message format and publish
        self.path_pub.publish(self.rrt_to_ros_path(path))
        ##### YOUR CODE ENDS HERE #####

if __name__ == '__main__':
    rospy.init_node('rrt_planner_node')
    
    rrtp = RRTPlanner()
    
    rospy.spin()
    

