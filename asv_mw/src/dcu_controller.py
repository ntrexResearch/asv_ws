#! /usr/bin/python

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
from std_msgs.msg import Float32, Int32, Int16

from serial_thread import DCUSerialThread,  queue_handler
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped
from definition import *

# Constant
wheel_to_wheel_d = 0.56 #0.29 # unit m
wheel_radius = 0.15
pulse_per_rev = 2000 #unit pulse/1 revolution
pulse_per_distance = pulse_per_rev / (2 * pi * wheel_radius) # pulse_per_rev / distance_per_rev ~ 30437.3673,,,

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
# 1 means manual; 0 means auto
mode = 1

def init_odom():
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
    speed = 0
    if linear > max_lin_speed:
        speed = max_lin_speed
    elif linear < min_lin_speed:
        speed  = min_lin_speed
    else:
        speed = linear
    return speed

def limit_steering_angle(angle):
    steering_angle = 0
    if angle > max_steering:
        steering_angle = max_steering
    elif angle < min_steering:
        steering_angle = min_steering
    else:
        steering_angle = angle
    return steering_angle

def on_new_ackermann(data):
    global mode
    global emergency_flag
    global front_obstacle_flag
    global rear_obstacle_flag
    #if not auto_mode or emergency_flag or front_obstacle_flag or rear_obstacle_flag:

    lin_speed_limited = limit_lin_speed(data.drive.speed)
    steering_angle_degree = data.drive.steering_angle * degree_per_rad
    steering_angle_limited = limit_steering_angle(steering_angle_degree)
    #print(data.drive.steering_angle, steering_angle_limited)
    lin_vel_rpm = int( lin_speed_limited * asv_ms_to_rpm )  # 60s per min
    
    #if mode:
    #    remote_tx_queue.put('c=0,0\r\n')
    # Check the status and then publish
    #else:

    #print("Command working")
    #print(lin_vel_rpm, data.drive.speed, data.drive.steering_angle)
    remote_tx_queue.put('c=' + str(lin_vel_rpm)+ ',' + str(int(steering_angle_limited)) + '\r\n')

    #remote_tx_queue.put('c=30,' + str(data.drive.steering_angle) + '\r\n')
def on_new_cmd(data):
    global remote_tx_queue
    remote_tx_queue.put('co1='+ str(data.data)+';co2='+ str(data.data)+'\r\n')

def shutdownhook():
    global ctrl_c
    global remote_tx_queue
    global serialThread
    ctrl_c = True
    # Stop the wheels
    remote_tx_queue.put('c=0,0\r\n') 
    serialThread.join()

if __name__ == "__main__":
    rospy.init_node("dcu_controller_node") 

    port = rospy.get_param("~serial_dev")
    tx_queue = Queue.Queue()
    remote_tx_queue = Queue.Queue()
    rx_queue = Queue.Queue()
    serialThread = DCUSerialThread(1, "serialThread-1", remote_tx_queue,tx_queue, rx_queue, port)
    rate = rospy.Rate(20)
    # On shutdown stop the motor and close the serial port
    rospy.on_shutdown(shutdownhook)

    publisher_mw_fault1 = rospy.Publisher("/mw/fault1", Int32, queue_size=10)
    publisher_mw_fault2 = rospy.Publisher("/mw/fault2", Int32, queue_size=10)
    subscriber_cmd = rospy.Subscriber("mw/command", Int32, on_new_cmd, queue_size=10)
    subscriber_ackermann = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped, on_new_ackermann, queue_size=10)

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
        #print("what")a

        if tx_queue.empty():
            tx_queue.put('s\r\n')
            #print("work")
        #print(rx_queue.empty())
        while not rx_queue.empty():
            rx_message = queue_handler(rx_queue, False)
            message_type = rx_message[0]
            #print(message_type)
            if message_type == "s":
                status = int(rx_message[1])
                mode = (status >> 3) & 0x1
                emergency_flag = not ((status >> 2) & 0x1)
                # front_obstacle_flag = (status >> 1) & 0x1
                # rear_obstacle_flag = status & 0x1
                #if not auto_mode or emergency_flag: or front_obstacle_flag or rear_obstacle_flag:
                #    break
                # when mode is 1, manual. when the mode is 0, it is auto.

                if mode or emergency_flag:
                    break
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
                    print("Left plus to minus flag")
                elif ( left_encoder_prev < -25000 and left_encoder_prev > min_encoder ) and (left_encoder > 25000 and left_encoder <= max_encoder ):
                    delta_left = (min_encoder - left_encoder_prev) - (max_encoder - left_encoder )
                    print("Left minus to plus flag")
                else:
                    delta_left = left_encoder - left_encoder_prev

                if ( right_encoder_prev > 25000 and right_encoder_prev <= max_encoder ) and ( right_encoder < -25000 and right_encoder >= min_encoder ):
                    delta_right = (max_encoder - right_encoder_prev) - (min_encoder - right_encoder) 
                    print("Right plus to minus flag")
                elif ( right_encoder_prev < -25000 and right_encoder_prev >= min_encoder ) and (right_encoder > 25000 and right_encoder <= max_encoder ):
                    delta_right = (min_encoder - right_encoder_prev) - (max_encoder - right_encoder )
                    print("Right minus to plus flag")
                else:
                    delta_right = right_encoder - right_encoder_prev

                # Block delta values over than 150
                if (abs(delta_left) > 150):
                    print("Delta outlier check")
                    print(delta_left, left_encoder, left_encoder_prev)
                    delta_left = delta_left_prev
                    
                if (abs(delta_right) > 150):
                    print("Delta outlier right check")
                    print(delta_right, right_encoder, right_encoder_prev)
                    delta_right = delta_right_prev


                delta_s = (delta_left - delta_right) / 2.0 / pulse_per_distance
                delta_th = (delta_right + delta_left) / wheel_to_wheel_d / pulse_per_distance
                delta_x = delta_s * cos(th + delta_th / 2.0)  # vx * cos(th) * dt
                delta_y = delta_s * sin(th + delta_th / 2.0)  # vx * sin(th) * dt
		
		vs = (left_velocity - right_velocity) / 2.0 / asv_ms_to_rpm
		vth = (left_velocity + right_velocity) / wheel_to_wheel_d / asv_ms_to_rpm

                current_time = rospy.Time.now()
                step_time = (current_time - last_time).to_sec()

                #print("Print ", step_time)
                #velocity_l = delta_left / step_time / pulse_per_distance
                #velocity_r = delta_right / step_time / pulse_per_distance
                #vs = (velocity_l + velocity_r) / 2.0
                #print(velocity_l)
                #print(velocity_r)
                #print("vs = ",vs)
                #vth = (velocity_r - velocity_l) / wheel_to_wheel_d * 2.0
                #print("vth = ",vth)
                ## delta_th = vth * dt
                x += delta_x
                y += delta_y
                th += delta_th
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
                # print("delta values: ")
                #print(delta_left, delta_right)
                #if abs(delta_left) > 1000 or abs(delta_right) > 1000:
                #   print("Check encoder values")
                #    print(left_encoder, left_encoder_prev)
                #    print(right_encoder, right_encoder_prev)
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
                #print("Publishing odom")
                # publish the message
                odom_pub.publish(odom)
                last_time = current_time
                left_encoder_prev = left_encoder
                right_encoder_prev = right_encoder
                delta_left_prev = delta_left
                delta_right_prev = delta_right
        # print(tx_queue.empty())
        rate.sleep()
