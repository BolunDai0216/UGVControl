import signal
import time

import numpy as np
import rospy
from gmm.msg import Commands
from ros_utils import UGVROSNode
from utils import get_p_control


def main():
    # Initialize the ROS node
    node = UGVROSNode()

    # Define variables and constants
    rate = rospy.Rate(100)
    error_threshold = 0.01
    control = Commands()
    _path = None
    path_time = None

    # Wait for the node to be spawned
    time.sleep(1.0)

    # Define a custom callback function
    def ctrl_c_handler(signum, frame):
        print("Ctrl+C pressed. Sending zero velocity.")

        for _ in range(100):
            control.header.stamp = rospy.Time.now()
            control.velocity = 0.0
            control.omega = 0.0
            node.publish(control)
            rate.sleep()

        exit()

    # Register the custom callback for the Ctrl+C signal (SIGINT)
    signal.signal(signal.SIGINT, ctrl_c_handler)

    # Wait till the first path is received
    while True:
        _path = node.path

        if _path is not None:
            path = _path
            path_time = path.header.stamp.to_time()
            pose_idx = 0
            pose_list_len = len(path.poses)
            break

    # Check if variables are initialized properly
    assert path is not None, "path should not be None"
    assert path_time is not None, "path_time should not be None"

    init_time = rospy.Time.now().to_time()

    # Main control loop
    while not rospy.is_shutdown():
        # retrieve the latest version of global planner COULD BE NONE!
        _path = node.path
        _path_time = _path.header.stamp.to_time()
        path_time_diff = _path_time - path_time

        # check if the path is updated and is not too old
        if path_time_diff >= 1e-3 and path_time_diff <= 20.0:
            path = node.path
            path_time = path.header.stamp.to_time()
            pose_idx = 0
            pose_list_len = len(path.poses)

        # TEST 1: See if always the path is the newest
        # print(path_time - init_time)

        # Get desired position
        x_des = path.poses[pose_idx].pose.position.x
        y_des = path.poses[pose_idx].pose.position.y
        p_des = np.array([x_des, y_des])

        # TEST 2: See if the node state is updated correctly
        # print(node.state)

        # set control
        _control, _, _ = get_p_control(node.state, p_des)
        control.header.stamp = rospy.Time.now()
        control.velocity = _control[0]
        control.omega = _control[1]

        # publish control
        node.publish(control)

        # if position error is small update desire position
        pos_error = np.linalg.norm(p_des - node.state[:2])

        # TEST 3: See if the desired position is updated correctly
        #         i.e., if the pos_error <= error_threshold will
        #         the desired position be updated?
        # print(pos_error, pose_idx, pose_list_len)

        # update desired position
        if pos_error < error_threshold:
            pose_idx = np.minimum(pose_idx + 1, pose_list_len - 1)

        rate.sleep()


if __name__ == "__main__":
    main()
