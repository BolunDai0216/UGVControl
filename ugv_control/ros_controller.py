import time

import numpy as np
import rospy
from ros_utils import UGVROSNode
from gmm.msg import Commands
from utils import get_p_control
import signal


def main():
    node = UGVROSNode()

    rate = rospy.Rate(100)
    data_time = None
    control = Commands()

    time.sleep(1.0)

    # Define a custom callback function
    def ctrl_c_handler(signum, frame):
        print("Ctrl+C pressed. Custom callback executed.")
        # Add your custom logic here
        control.header.stamp = rospy.Time.now()
        control.velocity = 0.0
        control.omega = 0.0
        node.publish(control)
        exit()
    
    # Register the custom callback for the Ctrl+C signal (SIGINT)
    signal.signal(signal.SIGINT, ctrl_c_handler)

    while True:
        _path = node.path

        if _path is not None:
            path = _path
            break

    while not rospy.is_shutdown():
        _path = node.path
        if data_time is None:
            data_time = path.header.stamp.to_time()

        new_data_time = _path.header.stamp.to_time()

        data_time_diff = new_data_time - data_time

        if data_time_diff >= 1e-3:
            path = node.path
            data_time = path.header.stamp.to_time()

            # if path is None:
            #     breakpoint()
        
        # breakpoint()
        try:
            print(path.poses[-1].pose.position.x, path.poses[-1].pose.position.y)
        except:
            control.header.stamp = rospy.Time.now()
            control.velocity = 0.0
            control.omega = 0.0

        # # check if the path is updated and is not too old
        # if data_time_diff > 1e-3:
        #     x_des = path.poses[-1].pose.position.x
        #     y_des = path.poses[-1].pose.position.y
        #     p_des = np.array([x_des, y_des])

        #     breakpoint()

            # set control
            # _control, _, _ = get_p_control(node.state, p_des)
            # control.header.stamp = rospy.Time.now()
            # control.velocity = _control[0]
            # control.omega = _control[1]

            # # publish control
            # node.publish(control)

        # if time_diff >= 2e-1:
        #     rospy.logerr("Path is too old. Stopping the robot.")
        #     break
        
        # rate.sleep()


if __name__ == '__main__':
    main()