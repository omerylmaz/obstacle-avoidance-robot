#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
from geometry_msgs.msg import Twist
import rospy, math, time

#Publishers
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.init_node('move_tea_distributor', anonymous=False)

def enviar_velocidad(vx,vy,vz,vaz):
    vel_msg = Twist()
    vel_msg.linear.x = float(vx)
    vel_msg.linear.y = float(vy)
    vel_msg.linear.z = float(vz)
    vel_msg.angular.z = float(vaz)
    vel_msg.angular.x = float(0.0)
    vel_msg.angular.y = float(0.0)
    vel_pub.publish(vel_msg)

def hover_pub():
    enviar_velocidad(0.0,0.0,0.0,0.0)

def takeoff_fun():
    takeoff_pub.publish(Empty())

def up_fun():
    vel_msg = Twist()
    vel_msg.linear.z = float(1.0)
    vel_pub.publish(vel_msg)

def down_fun():
    vel_msg = Twist()
    vel_msg.linear.z = float(-1.0)
    vel_pub.publish(vel_msg)

def forward_fun():
    vel_msg = Twist()
    vel_msg.linear.x = float(1.0)
    vel_pub.publish(vel_msg)

def backward_fun():
    vel_msg = Twist()
    vel_msg.linear.x = float(-1.0)
    vel_pub.publish(vel_msg)

def right_fun():
    vel_msg = Twist()
    vel_msg.linear.y = float(-1.0)
    vel_pub.publish(vel_msg)

def left_fun():
    vel_msg = Twist()
    vel_msg.linear.y = float(1.0)
    vel_pub.publish(vel_msg)

def cw_fun():
    vel_msg = Twist()
    vel_msg.angular.z = float(-1.0)
    vel_pub.publish(vel_msg)

def ccw_fun():
    vel_msg = Twist()
    vel_msg.angular.z = float(1.0)
    vel_pub.publish(vel_msg)

while True:
    time.sleep(1)
    up_fun()
    time.sleep(1)
    hover_pub()
    time.sleep(1)
    forward_fun()
    time.sleep(1)
    hover_pub()
    
