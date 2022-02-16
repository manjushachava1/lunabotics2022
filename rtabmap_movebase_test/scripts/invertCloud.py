import rospy, math, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import path_planner, numpy


class invertCloud:
    global pubCloud
    pubCloud = rospy.Publisher('/new_ground', PointCloud2, queue_size=1)

def __init__(self):
    rospy.init_node('invertcloud', anonymous=True)

    sub_ground = rospy.Subscriber('/rtabmap/cloud_ground', PointCloud2, self.invert_points)


def invert_points(self, msg):
    curr_cloud = msg


if __name__ == '__main__':
    invertCloud().run()

