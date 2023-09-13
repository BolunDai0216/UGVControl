import signal
import time

import rospy
from gmm.msg import Commands
from ros_utils import UGVROSNode


def main():
    node = UGVROSNode()

    rate = rospy.Rate(100)
    control = Commands()

    # Define a custom callback function
    def ctrl_c_handler(signum, frame):
        print("Ctrl+C pressed. Sending zero velocity.")

        for _ in range(100):
            control.header.stamp = rospy.Time.now()
            control.velocity = 0.0
            control.omega = 0.0
            node.publish(control)
            time.sleep(0.01)

        exit()

    # Register the custom callback for the Ctrl+C signal (SIGINT)
    signal.signal(signal.SIGINT, ctrl_c_handler)

    # Wait for the node to be spawned
    time.sleep(1.0)

    while not rospy.is_shutdown():
        control.header.stamp = rospy.Time.now()
        control.velocity = 0.1
        control.omega = 0.0

        # publish control
        node.publish(control)

        # sleep for 10ms
        rate.sleep()


if __name__ == "__main__":
    main()
