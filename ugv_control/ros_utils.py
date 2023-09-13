import rospy
from nav_msgs.msg import Path
# from geometry_msgs.msg import Twist
# from ugv_msgs.msg import cmd_vel 
# import Commands
from gmm.msg import Commands

class UGVROSNode:
    def __init__(self):
        # initialize the ROS node
        rospy.init_node('path_subscriber_node', anonymous=True)

        # store the path
        self.path = None

        # create a subscriber for the /local_planner topic
        self.subscriber = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.callback)

        # TODO: add tf subscriber parent: map, child: agent_base

        # create a publisher for the /cmd_vel topic
        self.publisher = rospy.Publisher('/ap/cmd_vel', Commands, queue_size=10)

    def callback(self, data):
        self.path = data
    
    def publish(self, cmd_vel):
        self.publisher.publish(cmd_vel)