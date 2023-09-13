from copy import deepcopy

import numpy as np
import rospy
import tf
from gmm.msg import Commands
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation


class UGVROSNode:
    def __init__(self):
        # initialize the ROS node
        rospy.init_node("path_subscriber_node", anonymous=True)

        # store the path
        self.path = None
        self.state = None

        # create a subscriber for the /local_planner topic
        self.subscriber = rospy.Subscriber(
            "/move_base/GlobalPlanner/plan", Path, self.callback
        )

        # TF subscriber with parent frame being /map and child frame being /agent_base
        self.listener = tf.TransformListener()
        self.parent_frame = "/map"
        self.child_frame = "/agent_base"
        self.listener.waitForTransform(
            self.parent_frame, self.child_frame, rospy.Time(), rospy.Duration(1.0)
        )

        # create a publisher for the /cmd_vel topic
        self.publisher = rospy.Publisher("/ap/cmd_vel", Commands, queue_size=10)

    def callback(self, data):
        self.path = data
        self.retrive_state()

    def publish(self, cmd_vel):
        self.publisher.publish(cmd_vel)

    def tf_2_pose(self, trans, quat):
        x = trans[0]
        y = trans[1]
        yaw = Rotation.from_quat(quat).as_rotvec()[2]

        return x, y, yaw

    def get_state(self):
        self.retrive_state()

        return deepcopy(self.state)

    def retrive_state(self):
        trans, quat = self.listener.lookupTransform(
            self.parent_frame, self.child_frame, rospy.Time(0)
        )
        x, y, yaw = self.tf_2_pose(trans, quat)

        self.state = np.array([x, y, yaw])
