import time

import numpy as np
import rospy
from ros_utils import UGVROSNode
from ugv_msgs.msg import cmd_vel
from utils import get_p_control


def main():
    node = UGVROSNode()

    rate = rospy.Rate(100)
    data_time = None
    control = cmd_vel()

    time.sleep(1.0)

    while not rospy.is_shutdown():
        # get path from subscriber
        path = node.path
        
        if data_time is None:
            data_time = path.header.stamp.to_time()

        new_data_time = path.header.stamp.to_time()
        current_time = rospy.Time.now().to_time()

        data_time_diff = new_data_time - data_time
        time_diff = current_time - data_time

        # check if the path is updated and is not too old
        if data_time_diff > 1e-3 and time_diff < 2e-1:
            x_des = path.poses[0].pose.position.x
            y_des = path.poses[0].pose.position.y
            p_des = np.array([x_des, y_des])

            # set control
            _control, _, _ = get_p_control(node.state, p_des)
            control.header.stamp = rospy.Time.now()
            control.velocity = _control[0]
            control.omega = _control[1]

            # publish control
            node.publish(control)

        if time_diff >= 2e-1:
            rospy.logerr("Path is too old. Stopping the robot.")
            break
        
        rate.sleep()


if __name__ == '__main__':
    main()