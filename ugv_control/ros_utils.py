import rospy
from nav_msgs.msg import Path
from ugv_msgs.msg import cmd_vel


class UGVROSNode:
    def __init__(self):
        # initialize the ROS node
        rospy.init_node('path_subscriber_node', anonymous=True)

        # store the path
        self.path = None

        # create a subscriber for the /local_planner topic
        self.subscriber = rospy.Subscriber('/local_planner', Path, self.callback)

        # create a publisher for the /cmd_vel topic
        self.publisher = rospy.Publisher('/cmd_vel', cmd_vel, queue_size=10)

    def callback(self, data):
        self.path = data
    
    def publish(self, cmd_vel):
        self.publisher.publish(cmd_vel)