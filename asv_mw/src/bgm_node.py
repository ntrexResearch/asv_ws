#! /usr/bin/env python

import roslib; roslib.load_manifest('sound_play')
import rospy
import actionlib
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal, SoundRequestResult, SoundRequestFeedback

from asv_mw.msg import Asv_state
import os
from std_msgs.msg import Int8

worship_bgm = "/worship.wav"
fault_sound = "/say-beep.wav"
client = None
bgm_signal =0


def feedback_cb(data):
    global bgm_signal
    global client
    if not bgm_signal:
        client.cancel_goal()
    #print(data)


def state_cb(data):
    global bgm_signal
    if data.mode is 1:
        bgm_signal = 1
    elif data.mode is 0:
        bgm_signal = 0


def sound_play_client(volume=1.0):
    global client
    client = actionlib.SimpleActionClient('sound_play', SoundRequestAction)

    client.wait_for_server()
    sub = rospy.Subscriber('/status', Asv_state, state_cb)

    print "Wav"
    goal = SoundRequestGoal()
    goal.sound_request.sound = SoundRequest.PLAY_FILE
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.arg = os.path.join(roslib.packages.get_pkg_dir('sound_play'),'sounds') + "/classic.wav"
    goal.sound_request.volume = volume

    client.send_goal(goal, feedback_cb=feedback_cb)
    result = SoundRequestResult()
    while 1:
        client.wait_for_result()
        print "test"
        result = client.get_result()
        print "Now restarting the loop"
        if result.playing == False:
            client.send_goal(goal, feedback_cb=feedback_cb)


if __name__ == '__main__':
    rospy.init_node('bgm_node')
    sound_play_client()
