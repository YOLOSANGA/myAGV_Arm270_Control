#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import time
import select
import threading

# Global interrupt flag
stop_flag = False

def key_input_thread():
    """
    Separate thread to monitor keyboard input in non-blocking mode.
    Sets stop_flag to True when 'q' or ESC is pressed.
    """
    global stop_flag
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while not stop_flag:
            rlist, _, _ = select.select([fd], [], [], 0.01)
            if rlist:
                key = sys.stdin.read(1)
                if key == 'q' or ord(key) == 27:  # ESC
                    stop_flag = True
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def move_agv_loop():
    global stop_flag
    rospy.init_node('agv_auto_forward_backward')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    print("AGV forward-backward loop started. Press 'q' or 'ESC' to stop.")

    # Start the key input thread
    key_thread = threading.Thread(target=key_input_thread, daemon=True)
    key_thread.start()

    while not rospy.is_shutdown() and not stop_flag:
        # Move forward
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.0
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < 3 and not stop_flag and not rospy.is_shutdown():
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        cmd_vel_pub.publish(Twist())
        time.sleep(0.5)

        # Move backward
        move_cmd.linear.x = -0.2
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < 3 and not stop_flag and not rospy.is_shutdown():
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        cmd_vel_pub.publish(Twist())
        time.sleep(0.5)

    # Stop the robot when exiting
    cmd_vel_pub.publish(Twist())
    print("AGV stopped and program terminated.")

if __name__ == "__main__":
    try:
        move_agv_loop()
    except rospy.ROSInterruptException:
        pass
