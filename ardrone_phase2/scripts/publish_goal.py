#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

import math


# Initialize ROS node
rospy.init_node('goal_pub')

# Initialize publisher
goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
start_pub = rospy.Publisher('command/trajectory', MultiDOFJointTrajectory, queue_size=10)

# Initialize service
rospy.wait_for_service('/gazebo/unpause_physics')
try:
    unpause_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e

# Initialize start
pt = MultiDOFJointTrajectoryPoint()
# Initialize position
t = Transform()
t.translation.x = rospy.get_param('~x_start', 0.)
t.translation.y = rospy.get_param('~y_start', 0.)
t.translation.z = rospy.get_param('~z_start', 0.)
a = rospy.get_param('~a_start', 0.)
t.rotation.x = 0.
t.rotation.y = 0.
t.rotation.z = math.sin(a/2.)
t.rotation.w = math.cos(a/2.)
pt.transforms.append(t)
# Initialize velocity and acceleration
pt.velocities.append(Twist())
pt.accelerations.append(Twist())
# Intialize trajectory
start = MultiDOFJointTrajectory()
start.header.frame_id = rospy.get_param('~frame_id', '/world')
start.points.append(pt)

# Initialize goal
goal = PoseStamped()
goal.header.frame_id = start.header.frame_id

goal.pose.position.x = rospy.get_param('~x_goal', 0.)
goal.pose.position.y = rospy.get_param('~y_goal', 0.)
goal.pose.position.z = rospy.get_param('~z_goal', 0.)

a = rospy.get_param('~a_goal', 0.)
goal.pose.orientation.x = 0.
goal.pose.orientation.y = 0.
goal.pose.orientation.z = math.sin(a/2.)
goal.pose.orientation.w = math.cos(a/2.)

# Ensure Gazebo is unpaused
unpaused = unpause_service()
i = 0
while i <= 10 and not unpaused:
    rospy.sleep(1.0)
    unpaused = unpause_service()
    i = i + 1

if not unpaused:
    rospy.logerr('Could not wake up Gazebo')
else:
    rospy.loginfo('Unpaused the Gazebo simulation')


start.header.stamp = rospy.Time.now()
start_pub.publish(start)

# Wait for 5 seconds to let Gazebo finish loading
rospy.sleep(5.0)

# Send goal
goal.header.stamp = rospy.Time.now()
goal_pub.publish(goal)

rospy.sleep(1.0)

