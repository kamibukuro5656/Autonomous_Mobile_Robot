#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from geometry_msgs.msg import Twist
import pygame

unit_linear = 0.35
unit_angular = 3.0
dead_zone = 0.2
speed_topic_name = 'speed_request'

def joystick_controller():
    pygame.joystick.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()

    pygame.init()

    rospy.init_node('joystick_controller', anonymous=True)

    unit_linear = rospy.get_param("unit_linear")
    unit_angular = rospy.get_param("unit_angular")
    dead_zone = rospy.get_param("dead_zone")
    speed_topic_name = rospy.get_param("speed_topic_name")

    pub = rospy.Publisher(speed_topic_name, Twist, queue_size=1)
    
    r = rospy.Rate(100)

    # Adder型のmessageのインスタンスを作る
    msg = Twist()
    while not rospy.is_shutdown():
        axis_1 = -joy.get_axis(1)
        axis_2 = joy.get_axis(3)
        if(0 < axis_1 < dead_zone):
            axis_1 = 0
        elif(dead_zone <= axis_1):
            axis_1 -= dead_zone
        elif(0 > axis_1 > -dead_zone):
            axis_1 = 0
        elif(-dead_zone > axis_1):
            axis_1 += dead_zone
        
        if(0 < axis_2 < dead_zone):
            axis_2 = 0
        elif(dead_zone <= axis_2):
            axis_2 -= dead_zone
        elif(0 > axis_2 > -dead_zone):
            axis_2 = 0
        elif(-dead_zone > axis_2):
            axis_2 += dead_zone

        msg.linear.x = axis_1 * unit_linear
        msg.angular.z = -axis_2 * unit_angular
        print("linear:" + str(msg.linear.x) + " angular:" + str(msg.angular.z))
        pub.publish(msg)

        r.sleep()
        pygame.event.pump()

    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

if __name__ == '__main__':
    try:
            joystick_controller()

    except rospy.ROSInterruptException: pass
