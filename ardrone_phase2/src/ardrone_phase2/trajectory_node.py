#!/usr/bin/env python

# ROS dependencies
import rospy
# Subscriber message dependencies
from nav_msgs.msg import Path
# Publisher message dependencies
from geometry_msgs.msg import Point, Quaternion, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
# Visualization message dependencies
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

import numpy as np

class TrajectoryPlanner:
    # robot information
    v_desired = None # desired velocity for the quadrotor
    
    # trajectory information
    corner_radius = None # radius of corners to cut
    dt = None # Time step for path
   
    # ROS objects
    path_sub = None # subscriber to get the path
    marker_pub = None # publisher to send marker array
    traj_pub = None # publisher to send the robot trajectory
    
    ##########
    # Constructor
    # Initializes a TrajectoryPlanner object
    def __init__(self):
        # Initialize map info using parameter server
        self.v_desired = rospy.get_param('~v_desired', 1.0)
        self.corner_radius = rospy.get_param('~corner_radius', 0.01)
        self.dt = rospy.get_param('~dt', 0.01)
        
        # Initialize ROS objects
        self.path_sub = rospy.Subscriber('path', Path, self.path_callback)
        self.marker_pub = rospy.Publisher('~/trajectory_markers', MarkerArray, queue_size=10)
        self.traj_pub = rospy.Publisher('command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    
    ##########
    # compute_coefficients
    # Takes in a nav_msgs/Path message and computes a set of polynomial coefficients
    # that determine the position as a function of time. The format of the output is
    # a numpy array of the duration of each segment and a list of numpy arrays
    # containing the polynomial coefficients.
    def compute_coefficients(self, msg):
        n_pts = len(msg.poses) # number of points in the path
        n_seg = n_pts - 1 # number of segments in the path
        # Convert nav_msgs/Path msg to numpy arrays
        path = np.zeros((n_pts, 3), dtype=float) # initialize numpy arrays
        yaw = np.zeros((n_pts, 1), dtype=float)
        i = 0
        for p in msg.poses:
            path[i,:] = [p.pose.position.x, p.pose.position.y, p.pose.position.z]
            yaw[i] = 2.*np.arctan2(p.pose.orientation.z, p.pose.orientation.w)
            i = i+1
        
        if path.shape[0] == 1: # Single point
            D = 0
            C = []
            C.append(path)
        elif path.shape[0] == 2: # Single line
            D = np.linalg.norm(path[1,:] - path[0,:])
        else: # Multi segment
            # Find distance between points
            d = np.diff(path, axis=0)
            d = np.linalg.norm(d, axis=1)
            D = np.cumsum(d)
            # Determine boundary conditions for curves
            ### Extend each section to smooth out trajectory rather than
            ### solving for full path.  Has effect of non-zero vel at start
            ### of each segment.    If segment is too short, set distance
            ### to be half of length.
            d = d.reshape((d.size, 1))
            d_before = np.copy(d[:-1]) # needed because by default uses a reference to original data
            d_after  = np.copy(d[1:])
            d_before[d_before < self.corner_radius * 2.]  = d_before[d_before < self.corner_radius * 2.] / 2.
            d_before[d_before >= self.corner_radius * 2.] = self.corner_radius
            d_after[d_after < self.corner_radius * 2.]    = d_after[d_after < self.corner_radius * 2.] / 2.
            d_after[d_after >= self.corner_radius * 2.]   = self.corner_radius
            # Position
            ### Extend each point by a distance before and after in
            ### direction of the segments.
            p_before =  path[1:-1, :] - \
                np.multiply( \
                  np.divide(path[1:-1, :] - path[:-2, :], \
                    np.matmul(d[:-1], np.ones((1,3)))), \
                  np.matmul(d_before, np.ones((1,3))))
            yaw_before = yaw[1:-1] - \
                np.multiply(np.divide(yaw[1:-1] - yaw[:-2], d[:-1]), d_before)
            p_after =  path[1:-1, :] + \
                np.multiply( \
                  np.divide(path[2:, :] - path[1:-1, :], \
                    np.matmul(d[1:], np.ones((1,3)))), \
                  np.matmul(d_after, np.ones((1,3))))
            yaw_after = yaw[1:-1] + \
                np.multiply(np.divide(yaw[2:] - yaw[1:-1], d[1:]), d_after)
            # Velocity
            ### Set desired velocity to be v at end points
            v_before = self.v_desired * np.divide(path[1:-1, :] - path[:-2, :], \
                    np.matmul(d[:-1], np.ones((1,3))))
            v_after  = self.v_desired * np.divide(path[2:, :] - path[1:-1, :], \
                    np.matmul(d[1:], np.ones((1,3))))
            # Acceleration
            a_before = np.zeros(p_before.shape)
            a_after  = np.zeros(p_after.shape)
            # Add marker for control points where segments start/end
            mp = Marker() # marker for points
            mp.header.frame_id = "/world"
            mp.header.stamp = rospy.Time.now()
            mp.ns = 'points'
            mp.id = 0
            mp.type = mp.POINTS
            mp.action = mp.ADD
            mp.scale.x = 0.05
            mp.scale.y = 0.05
            mp.color.a = 1.0
            mp.color.r = 0.0
            mp.color.g = 0.0
            mp.color.b = 1.0
            mp.points.append(Point(path[0,0], path[0,1], path[0,2])) # start of path
            for k in range(p_before.shape[0]): # points before/after each corner
                mp.points.append(Point(p_before[k,0], p_before[k,1], p_before[k,2]))
                mp.points.append(Point(p_after[k,0], p_after[k,1], p_after[k,2]))
            mp.points.append(Point(path[-1,0], path[-1,1], path[-1,2])) # end of path
            # Initialize marker array and add points marker to it
            ma = MarkerArray() # marker array for visualization
            ma.markers.append(mp)
            # Compute coefficients for 5th order polynomial for each segment and create a marker for each velocity vector
            DT = np.zeros((2*n_pts-3, 1))
            C = []
            for k in range(DT.shape[0]):
                # Create marker that will show the velocity at start point of the segment
                m = Marker()
                m.header.frame_id = "/world"
                m.header.stamp = rospy.Time.now()
                m.ns = 'velocity'
                m.id = k
                m.type = m.ARROW
                m.action = m.ADD
                m.scale.x = 0.02
                m.scale.y = 0.1
                m.color.a = 1.0
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                # For each segment, compute the duration, B matrix, and add points to visualization marker
                if k == 0: # First segment
                    m.points.append(Point(path[0,0], path[0,1], path[0,2])) # start of arrow
                    m.points.append(Point(path[0,0], path[0,1], path[0,2])) # end of arrow
                    ##### YOUR CODE STARTS HERE #####
                    # Compute duration of segment
                    DT[k,0] = 0.
                    # Compute coefficient matrix of boundary conditions
                    B = 0.
                    ##### YOUR CODE ENDS HERE #####
                elif k == (DT.size - 1): # Last segment
                    m.points.append(Point(p_after[-1,0], p_after[-1,1], p_after[-1,2])) # start of arrow
                    m.points.append(Point(p_after[-1,0]+v_after[-1,0], p_after[-1,1]+v_after[-1,1], p_after[-1,2]+v_after[-1,2])) # end of arrow
                    ##### YOUR CODE STARTS HERE #####
                    # Compute duration of segment
                    DT[k,0] = 0.
                    # Compute coefficient matrix of boundary conditionsB = 0.
                    ##### YOUR CODE ENDS HERE #####
                elif k % 2 == 0: # Straight segment
                    kk = (k-2)/2 # get index of point
                    m.points.append(Point(p_after[kk,0], p_after[kk,1], p_after[kk,2])) # start of arrow
                    m.points.append(Point(p_after[kk,0]+v_after[kk,0], p_after[kk,1]+v_after[kk,1], p_after[kk,2]+v_after[kk,2])) # end of arrow
                    ##### YOUR CODE STARTS HERE #####
                    # Compute duration of segment
                    DT[k,0] = 0.
                    # Compute coefficient matrix of boundary conditions
                    B = 0.
                    ##### YOUR CODE ENDS HERE #####
                else: # Curved segment
                    kk = (k-1)/2 # get index of point
                    m.points.append(Point(p_before[kk,0], p_before[kk,1], p_before[kk,2])) # start of arrow
                    m.points.append(Point(p_before[kk,0]+v_before[kk,0], p_before[kk,1]+v_before[kk,1], p_before[kk,2]+v_before[kk,2])) # end of arrow
                    ##### YOUR CODE STARTS HERE #####
                    # Compute duration of segment
                    DT[k,0] = 0.
                    # Compute coefficient matrix of boundary conditions
                    B = 0.
                    ##### YOUR CODE ENDS HERE #####
                # Add marker to marker array 
                ma.markers.append(m)
                if DT[k,0] > 1e-6: # if segment is of nonzero length, compute coefficients
                    ##### YOUR CODE STARTS HERE #####
                    # Compute A matrix of times
                    pass
                    # Compute solution to A*X = B and store in C array
                    pass
                    ##### YOUR CODE ENDS HERE #####
        DT = DT[DT > 1e-6] # only keep segments of nonzero length
        self.marker_pub.publish(ma) # publish visualization markers
        return (C, DT)
    
    ##########
    # compute_points
    # Takes the duration and polynomial coefficient data from compute_coefficients
    # and outputs a list of MultiDOFJointTrajectoryPoint objects containing the
    # pose, velocity, and acceleration data for the robot at each time step.
    def compute_points(self, C, DT):
        t_max = np.sum(DT) # total time for all segments
        t_start = np.concatenate((np.array([0.]), np.cumsum(DT[:-1]))) # get start time for each segment
        t_offset = 0. # offset between start of segment and first point added
        
        pts = [] # initialize empty array
        for k in range(len(C)):
            ##### YOUR CODE STARTS HERE #####
            # Extract position coefficients from C[k]
            px = 0
            py = 0
            pz = 0
            pa = 0
            # Compute velocity coefficients
            vx = 0
            vy = 0
            vz = 0
            va = 0
            # Compute acceleration coefficients
            ax = 0
            ay = 0
            az = 0
            aa = 0
            # Evaluate at each time step
            for t in np.arange(t_offset, DT[k], self.dt): # step through time of segment k
                # Create empty MultiDOFJointTrajectoryPoint object
                pt = MultiDOFJointTrajectoryPoint()
                # Compute and fill in pose
                
                # Compute and fill in velocity
                
                # Compute and fill in acceleration
                
                # Compute and fill in time_from_start
                
                # Append to list of points
                pts.append(pt)
            # MEE 5411 only: Compute new offset time to ensure even spacing between all points
            if k < len(C)-1:
                t_offset = 0.
            ##### YOUR CODE STARTS HERE #####
        # Return data
        return pts
    
    ##########
    # path_callback
    # Callback function for the path subscriber. Takes in the path message and calls
    # compute_coefficients and compute_points to fill in the MultiDOFJointTrajectory
    # expected by the drone simulation
    def path_callback(self, msg):
        rospy.logdebug('TrajectoryPlanner: Got path')
        # Compute polynomial coefficients of the trajectory
        (C, DT) = self.compute_coefficients(msg)
        # Initialize trajectory msg
        traj = MultiDOFJointTrajectory()
        traj.header.frame_id = msg.header.frame_id
        traj.header.stamp = rospy.Time.now() + rospy.Duration(1.)
        traj.joint_names = ['ardrone']
        # Compute points in the trajectory
        traj.points = self.compute_points(C, DT)
        # Publish trajectory
        self.traj_pub.publish(traj)

if __name__ == '__main__':
    rospy.init_node('trajectory_planner_node')
    tp = TrajectoryPlanner()
    rospy.spin()

