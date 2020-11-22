import rospy
from drive_ros_msgs import *
from geometry_msgs.msg import Pose2D



def talker():
    pub = rospy.Publisher("trajectory_publisher", Trajectory, queue_size=10)






def get_round():
    straigth_point=[ Pose2D(x,0, 0) for x in range(50) ]
    