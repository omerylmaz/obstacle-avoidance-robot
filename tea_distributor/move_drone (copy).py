#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
rospy.init_node('move_tea_distributor', anonymous=False)

obstacle_detected = False

def enviar_velocidad(vx, vy, vz, vaz):
    global obstacle_detected
    if obstacle_detected:
        vx = 0
        vy = 0
        vaz = 0.5
        rospy.logwarn("Obstacle detected! Turning...")
    else:
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
        rospy.logwarn("Obstacle detected detected.")
    else:
        obstacle_detected = False


scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

def takeoff_sequence():
    enviar_velocidad(0, 0, 1, 0)
    time.sleep(5)
    enviar_velocidad(0, 0, 0, 0)
    time.sleep(1)

def movement_sequence():
    enviar_velocidad(0, 0, 1, 0)
    time.sleep(1)
    while not rospy.is_shutdown():
        time.sleep(1)
        enviar_velocidad(1, 0, 0, 0)
        time.sleep(1)
        time.sleep(1)

if __name__ == '__main__':
    takeoff_sequence()
    movement_sequence()
