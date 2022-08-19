#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/catvehicle/camera_front/image_raw_front',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/catvehicle/cmd_vel_safe',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # change below lines to map the color you wanted robot to follow
        # lower_yellow = numpy.array([10, 10, 10])
        # upper_yellow = numpy.array([255, 255, 250])

        lower_black = numpy.array([0, 0, 0], dtype="uint8")
        upper_black = numpy.array([360, 255, 50], dtype="uint8")

        # mask = cv2.inRange(image, lower_white, upper_white)
        # mask = cv2.inRange(hsv, lower_white, upper_white)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        h, w, d = image.shape
        search_top = 3 * h / 4 - 25
        search_bot = 3 * h / 4 + 20
        mask[0:int(search_top), 0:w] = 0
        mask[int(search_bot):h, 0:w] = 0
        M = cv2.moments(mask)

        # Turn around if an obstacle is detected.

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            cxm = cx + 84  # 120#143 #CW
            if cx <= 2 * h / 8:
                cxm = int(cx + (h / 2))

            cv2.circle(image, (cxm, cy), 20, (0, 0, 255), -1)
            # CONTROL starts
            err = cxm - w / 2
            self.twist.linear.x = 9.3
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # CONTROL ends

        resized_image = cv2.resize(image, (620, 360))
        # resized_mask = cv2.resize(mask, (620, 360))

        # cv2.imshow("masked output", resized_mask)
        cv2.imshow("normal output", resized_image)
        # cv2.im
        cv2.waitKey(3)

    # ef sendCommand(cx):
    # global curve

    # lr = (cx - w // 2) // sensitivity
    # lr = int(np.clip(lr, -10, 10))


def main():
    Follower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    rospy.init_node('line_following_node', anonymous=True)
    main()

# Edited by Muhammad Hafiz Haikal (hafeez.haikal) ~instagram hehe
