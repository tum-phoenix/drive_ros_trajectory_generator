import rospy
from drive_ros_msgs.msg import *
from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Twist2D


def talker():
    pub = rospy.Publisher("trajectory_publisher", Trajectory, queue_size=10)
    rospy.init_node('PUB', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    gen=drive_straight()
    while not rospy.is_shutdown():
        pub.publish(next(gen))
        rate.sleep()


def get_straight():
    straigth_point_x=[ Pose2D(x,0, 0) for x in range(500) ]
    straigth_point_v=[ Twist2D(x,0, 0) for x in range(500) ]
    return straigth_point_x,straigth_point_v


def drive_straight():
    pose, veleocity = get_straight()
    for i in range(490):
        yield Trajectory([TrajectoryPoint(pose[j],veleocity[j]) for j in range(i,i+10)])


if __name__ == "__main__":
    talker()