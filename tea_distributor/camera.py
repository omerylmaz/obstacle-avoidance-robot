#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # Görüntüyü al ve cv2 formatına çevir
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Görüntü işleme veya başka bir şey yap
        # Örneğin, görüntüyü ekrana göster
        cv2.imshow("Front Camera Image", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        print(e)

def main():
    rospy.init_node('image_listener', anonymous=True)

    # /front_cam/camera/image topic'ine abone ol
    rospy.Subscriber("/front_cam/camera/image", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

