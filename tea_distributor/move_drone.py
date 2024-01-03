#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time
import random

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
rospy.init_node('move_tea_distributor', anonymous=False)

obstacle_detected = False
turning = False

def enviar_velocidad(vx, vy, vz, vaz):
    global obstacle_detected, turning
    turn_direction = -1
    if obstacle_detected:
        if not turning:
            turning = True
            # turn_direction = random.choice([-1, 1])
            rospy.logwarn("Obstacle detected! Turning...")
            vel_msg = Twist()
            vel_msg.angular.z = 0.5 * turn_direction
            vel_pub.publish(vel_msg)
            time.sleep(2) 
            vel_msg.angular.z = 0.0
            vel_pub.publish(vel_msg)
            rospy.loginfo("Turn complete. Scanning for open space...")
        else:
            vel_msg = Twist()
            vel_msg.angular.z = 0.1 * turn_direction
            vel_pub.publish(vel_msg)
            time.sleep(2) 
            vel_msg.angular.z = 0.0
            vel_pub.publish(vel_msg)
            rospy.loginfo("yanlış ellerdesin bro...")
    else:
        turning = False
        rospy.loginfo("No obstacle detected. Moving forward...")
        vel_msg = Twist()
        vel_msg.linear.x = float(vx)
        vel_msg.linear.y = float(vy)
        vel_msg.linear.z = float(vz)
        vel_msg.angular.z = float(vaz)
        vel_pub.publish(vel_msg)

def scan_callback(data):
    global obstacle_detected
    center_index = len(data.ranges) // 2 
    angle_range = 30 

    min_index = center_index - angle_range
    max_index = center_index + angle_range
    center_ranges = data.ranges[min_index:max_index]

    filtered_ranges = [distance for distance in center_ranges if distance > 0.1]
    if not filtered_ranges:
        return

    range_ahead = min(filtered_ranges)
    rospy.loginfo(f"Closest valid obstacle distance: {range_ahead} meters")

    if range_ahead < 2.0:
        obstacle_detected = True
    else:
        obstacle_detected = False

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

def takeoff_sequence():
    rospy.loginfo("Starting takeoff...")
    enviar_velocidad(0, 0, 1, 0)
    time.sleep(5)
    enviar_velocidad(0, 0, 0, 0)
    rospy.loginfo("Takeoff complete.")

def movement_sequence():
    rospy.loginfo("Starting movement...")
    enviar_velocidad(0, 0, 1, 0)
    time.sleep(1)
    while not rospy.is_shutdown():
        enviar_velocidad(1, 0, 0, 0)
        time.sleep(1)

if __name__ == '__main__':
    takeoff_sequence()
    movement_sequence()
