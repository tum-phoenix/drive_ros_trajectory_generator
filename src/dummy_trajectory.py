import rospy
from drive_ros_msgs.msg import *
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Twist2D
import numpy as np


def talker():
    pub = rospy.Publisher("trajectory_publisher", Trajectory, queue_size=10)
    pub_global = rospy.Publisher("global_trajectory_publisher", Trajectory, queue_size=10)
    rospy.init_node('PUB', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    gen = drive_straight()
    while not rospy.is_shutdown():
        global_trajectory, local_trajectory = next(gen)
        pub.publish(local_trajectory)
        pub_global.publish(global_trajectory)
        rate.sleep()


def get_straight():
    straight_point_x = [np.array([x, 0, 0]) for x in range(50000)]
    straights_point_v = [np.array([1, 0, 0]) for _ in range(50000)]
    return straight_point_x, straights_point_v


def get_curves():
    straight_point_x = [np.array([x, 0.5*x, 0]) for x in range(50000)]
    straights_point_v = [np.array([1, 0, 0]) for _ in range(50000)]
    return straight_point_x, straights_point_v


def np_to_pose(pose):
    return Pose2D(pose[0],pose[1],pose[2])


def np_to_twist(pose):
    return Twist2D(pose[0],pose[1],pose[2])


def drive_straight():
    pose, velocity = get_straight()
    for i in range(49000):
        yield Trajectory([TrajectoryPoint(np_to_pose(pose[j]), np_to_twist(velocity[j])) for j in range(len(pose))]),Trajectory([TrajectoryPoint(np_to_pose(pose[j]-pose[i]), np_to_twist(velocity[j])) for j in range(i,i+10)])


if __name__ == "__main__":
    talker()