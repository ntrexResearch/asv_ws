#! /usr/bin/env python

import roslib; roslib.load_manifest('sound_play')
import rospy
import actionlib
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal, SoundRequestResult, SoundRequestFeedback

import os
from std_msgs.msg import Int8

worship_bgm = "/worship.wav"
fault_sound = "/say-beep.wav"
client = None
test =0
def done_cb(goalState):
    print ''

def feedback_cb(data):
    global test
    global client
    if test:
        client.cancel_goal()
    #print(data)


def test_cb(data):
    global test
    if data.data is 1:
        test = 1
    else:
        test = 0

def sound_play_client(volume=1.0):
    global client
    client = actionlib.SimpleActionClient('sound_play', SoundRequestAction)

    client.wait_for_server()
    sub = rospy.Subscriber('/test', Int8, test_cb)

    print "Wav"
    goal = SoundRequestGoal()
    goal.sound_request.sound = SoundRequest.PLAY_FILE
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.arg = os.path.join(roslib.packages.get_pkg_dir('sound_play'),'sounds') + "/NewVillageMoveCut.wav"
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

    client.wait_for_result()
    print client.get_result()
    print "End wav"
    print


if __name__ == '__main__':
    rospy.init_node('bgm_node')
    sound_play_client()
