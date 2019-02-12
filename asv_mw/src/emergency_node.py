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
    pub.publish(1)
    global rate
    setting = termios.tcgetattr(sys.stdin)
    while 1:
        key = get_key(setting)
        if key == control_keys['enter']:
            em_flag = Int8()
            em_flag.data = 1
            pub.publish(em_flag)
	    print_current_emg(1)
        elif key == control_keys['space']:
            em_flag = Int8()
            em_flag.data = 0
            pub.publish(em_flag)
            print_current_emg(0)
	elif key == '\x03' or key == '\x71':  # ctr-c or q
	    break
        else:
            continue
        rate.sleep()


def print_current_emg(state):
    rospy.loginfo('\x1b[1M\r*********************************************')
    rospy.loginfo("\x1b[1M\rEmergency state : %d" % (state))
    rospy.loginfo('\x1b[1M\r*********************************************')


def print_state():
    rospy.loginfo('\x1b[1M\r*********************************************')
    rospy.loginfo("\x1b[1M\rASV emergency node activated")
    rospy.loginfo('\x1b[1M\rUse Enter to enable emergency state')
    rospy.loginfo('\x1b[1M\rUse Space to disable emergency states')
    rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
    rospy.loginfo('\x1b[1M\r*********************************************')


if __name__ == '__main__':
    pub = rospy.Publisher('/mw/emg', Int8, queue_size=10)
    rospy.init_node('emergency_node')
    rate = rospy.Rate(10)
    try:
        print_state()
	key_loop()
    except rospy.ROSInterruptException:
        pass
