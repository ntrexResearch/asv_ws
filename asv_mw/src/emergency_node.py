#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
import sys, select, termios, tty

control_keys = {
    'space' : '\x20',
    'enter' : '\x0D',
}


def get_key(setting):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, setting)
    return key


def key_loop():
    global pub
    global rate
    setting = termios.tcgetattr(sys.stdin)
    while 1:
        key = get_key(setting)
        if key == control_keys['enter']:
            em_flag = Int8()
            em_flag.data = 1
            pub.publish(em_flag)
        elif key == control_keys['space']:
            em_flag = Int8()
            em_flag.data = 0
            pub.publish(em_flag)
        else:
            continue
        rate.sleep()


def emergency_node():
    pub = rospy.Publisher('/asv_emergency', Int8, queue_size=10)
    rospy.init_node('emergency_node')
    rate = rospy.Rate(10)
    rospy.loginfo('\x1b[1M\r*********************************************')
    rospy.loginfo("\x1b[1M\rASV emergency node activated")
    rospy.loginfo('\x1b[1M\r*********************************************')
    while not rospy.is_shutdown():
        key_loop()


if __name__ == '__main__':
    try:
        emergency_node()
    except rospy.ROSInterruptException:
        pass
