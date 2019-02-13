#! /usr/bin/python
#-*- coding: utf-8 -*-
# 한글 입력을 위한 엔코딩 명시
#################################################################################
# Copyright 2018 NTREX CO.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Ji #

from __future__ import division
import rospy
import Queue

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int8

from serial_thread import DCUSerialThread,  queue_handler
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped
from asv_mw.msg import Asv_state
from definition import *
from collections import deque

# 로봇 구동부에 관한 정보를 기재한다.
wheel_to_wheel_d = 0.56 # 바퀴와 바퀴 간 거리  [m]
wheel_radius = 0.15
pulse_per_rev = 2000  # 한바퀴 회전시 엔코더 펄스 카운트(이 값은 부착된 엔코더와 감속기를 고려해 정해진다.) [pulse / rev]
pulse_per_distance = pulse_per_rev / (2 * pi * wheel_radius) # 1m 이동시 엔코더 펄스 카운트 [pulse / m]

asv_ms_to_rpm = 60 / wheel_radius

degree_per_rad = 180 / pi

max_lin_speed = 0.075
min_lin_speed = -0.075
max_steering = 30
min_steering = -30

max_encoder = 32767
min_encoder = -32768


fault_flag = False
index = 0
# 0 means manual; 1 means auto
mode = 0
emergency_flag = 0
fds_flag = rds_flag = 0

def init_odom():
    """
    이 함수는 ros topic odom 에 대한 정보를 초기화한다.
    :return: None
    """
    global x
    global y
    global th

    global vs
    # global vy
    global vth
    global left_velocity
    global right_velocity
    global left_encoder
    global right_encoder

    x = 0.0
    y = 0.0
    th = 0.0

    vs = 0.0
    # vy = 0
    vth = 0.0
    left_velocity = 0
    right_velocity = 0
    left_encoder = 0
    right_encoder = 0


def limit_lin_speed(linear):
    """
    이 함수는 로봇의 스피드를 제한한다.
    :param linear: 직진 스피드값을 받아온다.
    :return: 제한 처리가 된 스피드값을 내보낸다.
    """
    speed = 0
    if linear > max_lin_speed:
        speed = max_lin_speed
    elif linear < min_lin_speed:
        speed = min_lin_speed
    else:
        speed = linear
    return speed


def limit_steering_angle(angle):
    """
    이 함수는 로봇 조향각 입력을 제한한다.
    :param angle: 조향각 입력값을 받아온다.
    :return: 제한 처리가 된 조향각을 내보낸다.
    """
    steering_angle = 0
    if angle > max_steering:
        steering_angle = max_steering
    elif angle < min_steering:
        steering_angle = min_steering
    else:
        steering_angle = angle
    return steering_angle


def make_text_command(command_type, arg1=0.0, arg2=0.0):
    """
    이 함수를 사용해 DCU에 들어가는 text 기반 명령어를 만든다.
    :param command_type: DCU를 위한 명령어의 종류를 스트링으로 받는다.
    :param arg1: 주행 속도에 해당하는 명령값을 받는다.
    :param arg2: 조향각에 해당하는 명령값을 받는다.
    :return: 문자열로 된 DCU 명령어
    """
    if command_type == 'Control Motion':
        return 'c=%d,%d\r\n' % (arg1, arg2)
    elif command_type == 'Steer Control':
        return 'sc=%d\r\n' % (arg1)
    elif command_type == 'Poll Status':
        return 's\r\n'
    elif command_type == 'Emergency Control':
        return 'emg=%d\r\n' % (arg1)
    elif command_type == 'Distance Sensor Threshold':
        return 'sfdst\r\n'
    else:
        return " "


def on_new_ackermann(data):
    """
    이 함수는 ackermann_cmd topic 의 callback 함수이다.
    입력 값을 검토하고 DCU 시리얼 전송 queue에 명령어를 넣는다.
    :param data: ackermann_cmd data
    :return: None
    """
    global mode
    global emergency_flag
    global fds_flag
    global rds_flag
    global steer_fault
    #if not auto_mode or emergency_flag or front_obstacle_flag or rear_obstacle_flag:

    lin_speed_limited = limit_lin_speed(data.drive.speed)
    steering_angle_degree = data.drive.steering_angle * degree_per_rad
    steering_angle_limited = limit_steering_angle(steering_angle_degree)
    lin_vel_rpm = int( lin_speed_limited * asv_ms_to_rpm )
    # mode 가 1 이면 자율주행 모드 0이면 수동주행을 의미한다.
    # emergency_flag가 1이면 비상상태 0이면 정상을 의미한다.
    # fds_flag 와 rds_flag는 1이면 장애물이 있는 상태 0이면 없는 상태를 의미한다.
    if mode and not emergency_flag: # and not fds_flag and not rds_flag:
        if fds_flag is 1 and lin_vel_rpm >= 0:
            remote_tx_queue.put(make_text_command('Control Motion', 0, int(steering_angle_limited)))
        elif rds_flag is 1 and lin_vel_rpm <= 0:
            remote_tx_queue.put(make_text_command('Control Motion', 0, int(steering_angle_limited)))
        remote_tx_queue.put(make_text_command('Control Motion', lin_vel_rpm, int(steering_angle_limited)))
    # Check the status and then publish
    else:
        remote_tx_queue.put(make_text_command('Control Motion', 0, 0))


def on_new_cmd(data):
    """
    이 함수는 mw/command topic callback 함수이다.
    command 함수는 다음과 같은 기능을 수행한다.
    -- command 가 0이면 Motor control을 서보 오프시킨다.
    -- command 가 1이면 Motor control을 서보 온시킨다.
    -- command 가 2이면 Motor control에 발생한 fault를 clear한다.
    :param data: command topic 데이터이다.
    :return: None
    """
    global remote_tx_queue
    remote_tx_queue.put(make_text_command('Steer Control', data.data))


def on_new_emg(data):
    """
    이 함수는 mw/emg topic callback 함수이다.
    emg 가 1이면 비상정지 상태를 의미하고 emg 가 0이면 normal상태를 의미한다.
    :param data:
    :return:
    """
    global remote_tx_queue
    remote_tx_queue.put(make_text_command('Emergency Control', data.data))


def shutdownhook():
    """
    이 함수는 DCU 노드 종료시 불려지는 함수이다.
    :return: None
    """
    global ctrl_c
    global remote_tx_queue
    global serialThread
    ctrl_c = True
    # Stop the wheels
    remote_tx_queue.put(make_text_command('Control Motion', 0, 0)) 
    serialThread.join()


if __name__ == "__main__":
    rospy.init_node("dcu_controller_node")
    port = rospy.get_param("~serial_dev")
    tx_queue = Queue.Queue()
    remote_tx_queue = Queue.Queue()
    rx_queue = Queue.Queue()
    fds_deque = deque(maxlen=5)
    rds_deque = deque(maxlen=5)
    asv_state_info = Asv_state()
    serialThread = DCUSerialThread(1, "DCU serial thread", remote_tx_queue,tx_queue, rx_queue, port)
    rate = rospy.Rate(20)
    # On shutdown stop the motor and close the serial port
    rospy.on_shutdown(shutdownhook)

    publisher_asv_status = rospy.Publisher("/status", Asv_state, queue_size=1)
    publisher_mw_fault1 = rospy.Publisher("/mw/fault1", Int32, queue_size=10)
    publisher_mw_fault2 = rospy.Publisher("/mw/fault2", Int32, queue_size=10)
    subscriber_cmd = rospy.Subscriber("mw/steer_command", Int32, on_new_cmd, queue_size=10)
    subscriber_ackermann = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, on_new_ackermann, queue_size=10)
    subscriber_emg = rospy.Subscriber("mw/emg", Int8, on_new_emg, queue_size=1)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    init_odom()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    ctrl_c = False
    left_encoder_prev = 0
    right_encoder_prev = 0
    delta_left_prev = 0
    delta_right_prev = 0
    vs = 0
    vth = 0
    serialThread.start()
    while not ctrl_c:
        current_time = rospy.Time.now()
        if tx_queue.empty():
            tx_queue.put('s\r\n')
        while not rx_queue.empty():
            rx_message = queue_handler(rx_queue, False)
            message_type = rx_message[0]
            if message_type == "s":
                status = int(rx_message[1])
                steer_fault = (status >> 4)
                mode = (status >> 3) & 0x1
                emergency_flag = (status >> 2) & 0x1
                fds_state = (status >> 1) & 0x1
                rds_state = status & 0x1
                fds_deque.append(fds_state)
                rds_deque.append(rds_state)
                fds_flag = int(sum(fds_deque)/len(fds_deque))
                rds_flag = int(sum(rds_deque)/len(rds_deque))
                asv_state_info.mode = mode
                asv_state_info.emergency = emergency_flag
                asv_state_info.front_obstacle = fds_flag
                asv_state_info.rear_obstacle = rds_flag
                publisher_asv_status.publish(asv_state_info)
                # front_obstacle_flag = (status >> 1) & 0x1
                # rear_obstacle_flag = status & 0x1
                #if not auto_mode or emergency_flag: or front_obstacle_flag or rear_obstacle_flag:
                #    break
                # when mode is 1, manual. when the mode is 0, it is auto.
                #print(mode, emergency_flag)
                # 오토 모드 가 아니거나 비상 상태시 odometry를
                # if not mode or emergency_flag:
                #     break
                left_encoder = int(rx_message[2])
                right_encoder = int(rx_message[3])
                left_velocity = float(rx_message[4])
                right_velocity = float(rx_message[5])
                # For Ackermann Steering system, calculate the odometry as follows. 
                # The data to receive from DCU are as follows: left_velocity, right_velocity, left_encoder, right_encoder, steering_angle
                # Update odometry
                
                # For the special case when the encoder counter exceeds the range -32,769 ~ 32767
                if ( left_encoder_prev > 25000 and left_encoder_prev <= max_encoder ) and ( left_encoder < -25000 and left_encoder >= min_encoder ):
                    delta_left = (max_encoder - left_encoder_prev) - (min_encoder - left_encoder)
                    #print("Left plus to minus flag")
                elif ( left_encoder_prev < -25000 and left_encoder_prev > min_encoder ) and (left_encoder > 25000 and left_encoder <= max_encoder ):
                    delta_left = (min_encoder - left_encoder_prev) - (max_encoder - left_encoder )
                    #print("Left minus to plus flag")
                else:
                    delta_left = left_encoder - left_encoder_prev

                if ( right_encoder_prev > 25000 and right_encoder_prev <= max_encoder ) and ( right_encoder < -25000 and right_encoder >= min_encoder ):
                    delta_right = (max_encoder - right_encoder_prev) - (min_encoder - right_encoder) 
                    #print("Right plus to minus flag")
                elif ( right_encoder_prev < -25000 and right_encoder_prev >= min_encoder ) and (right_encoder > 25000 and right_encoder <= max_encoder ):
                    delta_right = (min_encoder - right_encoder_prev) - (max_encoder - right_encoder )
                    #print("Right minus to plus flag")
                else:
                    delta_right = right_encoder - right_encoder_prev

                # Block delta values over than 150
                if (abs(delta_left) > 150):
                    #print("Delta outlier check")
                    #print(delta_left, left_encoder, left_encoder_prev)
                    delta_left = delta_left_prev
                    
                if (abs(delta_right) > 150):
                    #print("Delta outlier right check")
                    #print(delta_right, right_encoder, right_encoder_prev)
                    delta_right = delta_right_prev


                delta_s = (delta_left - delta_right) / 2.0 / pulse_per_distance
                delta_th = (delta_right + delta_left) / wheel_to_wheel_d / pulse_per_distance
                delta_x = delta_s * cos(th + delta_th / 2.0)  # vx * cos(th) * dt
                delta_y = delta_s * sin(th + delta_th / 2.0)  # vx * sin(th) * dt
                vs = (left_velocity - right_velocity) / 2.0 / asv_ms_to_rpm
                vth = (left_velocity + right_velocity) / wheel_to_wheel_d / asv_ms_to_rpm
                current_time = rospy.Time.now()
                x += delta_x
                y += delta_y
                th += delta_th
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
                odom_broadcaster.sendTransform(
                    (x, y, 0),
                    odom_quat,
                    current_time,
                    "base_footprint",
                    "odom"
                )

                odom = Odometry()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_footprint"
                odom.header.stamp = current_time
                # set the position
                odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
                # set the velocity
                odom.twist.twist = Twist(Vector3(vs, 0, 0), Vector3(0, 0, vth))
                odom_pub.publish(odom)
                left_encoder_prev = left_encoder
                right_encoder_prev = right_encoder
                delta_left_prev = delta_left
                delta_right_prev = delta_right
        rate.sleep()
