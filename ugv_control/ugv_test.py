import time

import rospy
from ros_utils import UGVROSNode
from ugv_msgs.msg import cmd_vel


def main():
    node = UGVROSNode()

    rate = rospy.Rate(100)
    start_time = rospy.Time.now().to_time()
    control = cmd_vel()

    time.sleep(1.0)

    while not rospy.is_shutdown():
        control.header.stamp = rospy.Time.now()
        control.velocity = 0.1
        control.omega = 0.0

        # publish control
        node.publish(control)

        time_diff = rospy.Time.now().to_time() - start_time

        if time_diff >= 5.0:
            rospy.loginfo("Terminating Controller...")
            break
        
        rate.sleep()


if __name__ == '__main__':
    main()