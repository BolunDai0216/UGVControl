import time

import rospy
from ros_utils import UGVROSNode
# from ugv_msgs.msg import cmd_vel
from gmm.msg import Commands
import signal


def main():
    node = UGVROSNode()

    # rate = rospy.Rate(100)
    start_time = rospy.Time.now().to_time()
    control = Commands()

    # Define a custom callback function
    def ctrl_c_handler(signum, frame):
        print("Ctrl+C pressed. Custom callback executed.")
        # Add your custom logic here
        for i in range(100):
            control.header.stamp = rospy.Time.now()
            control.velocity = 0.0
            control.omega = 0.0
            node.publish(control)
            time.sleep(0.01)
        exit()
    
    # Register the custom callback for the Ctrl+C signal (SIGINT)
    signal.signal(signal.SIGINT, ctrl_c_handler)

    time.sleep(1.0)

    while not rospy.is_shutdown():
        control.header.stamp = rospy.Time.now()
        control.velocity = 0.0
        control.omega = 0.0
        # print(control)
        # publish control
        node.publish(control)



        # time_diff = rospy.Time.now().to_time() - start_time

        # if time_diff >= 5.0:
        #     rospy.loginfo("Terminating Controller...")
        #     break

        
        # rate.sleep()


if __name__ == '__main__':
    main()