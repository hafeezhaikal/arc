#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np

from function import *
from math import radians
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from time import sleep


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/catvehicle/camera_front/image_raw_front',
                                          Image, self.image_callback)

        self.add_sub = rospy.Subscriber('catvehicle/camera_front/image_raw_front',
                                        Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('/catvehicle/cmd_vel_safe',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv, 7)

        # change below lines to map the color you wanted robot to follow
        # lower_yellow = numpy.array([10, 10, 10])
        # upper_yellow = numpy.array([255, 255, 250])

        # lower_black = numpy.array([0, 0, 0], dtype="uint8")
        # upper_black = numpy.array([360, 255, 50], dtype="uint8")

        # mask = cv2.inRange(image, lower_white, upper_white)
        # mask = cv2.inRange(hsv, lower_white, upper_white)
        mask = cv2.inRange(hsv, np.array([100, 8, 27]), np.array([117, 50, 52]))
        mask[0:110, :] = 0
        points, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        sorted_points = sorted(points, key=len)

        right_lane_mask = cv2.fillPoly(np.zeros((256, 256)), pts=[sorted_points[-1]], color=(255))
        left_lane_mask = cv2.fillPoly(np.zeros((256, 256)), pts=[sorted_points[-2]], color=(255))
        obstacle_mask = cv2.inRange(image, np.array([70, 4, 93]), np.array([115, 16, 150]))
        # obstacle_mask2 = cv2.inRange(frame, np.array([104,88,77]), np.array([255,211,93]))
        obstacle_res = obstacle_mask
        # obstacle_res = cv2.bitwise_or(obstacle_mask, obstacle_mask2)
        yellow_mask = cv2.inRange(hsv, np.array([28, 115, 154]), np.array([31, 180, 255]))

        points, _ = cv2.findContours(obstacle_res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        sorted_points = sorted(points, key=len)
        try:
            if cv2.contourArea(sorted_points[-1]) > 25:
                x, y, w, h = cv2.boundingRect(sorted_points[-1])
                mean_obstacle = np.mean(np.where(obstacle_res[y:y + h, x:x + w] > 0), axis=1)[1]

                yellow_roi = np.mean(np.where(yellow_mask[y:y + h, :] > 0), axis=1)[1]

                if mean_obstacle < yellow_roi:
                    print('obstacle is on left')
                else:
                    print('obstacle is on right')

                frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        except:
            pass

        CURRENT_PXL = np.mean(np.where(right_lane_mask[120:150, :] > 0), axis=1)[1]
        SECOND_PXL = np.mean(np.where(left_lane_mask[120:150, :] > 0), axis=1)[1]

        if np.isnan(CURRENT_PXL): CURRENT_PXL = 128
        if np.isnan(SECOND_PXL): SECOND_PXL = 128

        direction_s, slope = detect_yellow_line(image)

        if slope < 0:
            position = 'left'
        else:
            position = 'right'

        cv2.putText(direction_s, position, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # ImShow
        show_img = np.concatenate((image, direction_s), axis=1)
        h1_axis = np.concatenate((left_lane_mask, right_lane_mask), axis=1)
        h2_axis = np.concatenate((obstacle_res, yellow_mask), axis=1)
        show_mask = np.concatenate((h1_axis, h2_axis), axis=0)

        cv2.imshow('img', show_img)
        cv2.imshow('info', show_mask)
        cv2.waitKey(3)

    # ef sendCommand(cx):
    # global curve

    # lr = (cx - w // 2) // sensitivity
    # lr = int(np.clip(lr, -10, 10))

    # def object_avoidance(self, msg):
    #  print(msg.range[360])
    # self.twist.linear.x = 0.1
    # if msg.range[360] < 1:
    # self.twist.linear.x = 3
    # self.twist.angular.z = radians(75)
    # self.cmd_vel_pub.publish(self.twist)

    # resized_image = cv2.resize(debug_img, (620, 360))
    # resized_mask = cv2.resize(mask, (620, 360))

    # cv2.imshow("masked output", resized_mask)
    # cv2.imshow("normal output", resized_image)
    # cv2.waitKey(3)


def main():
    Follower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    rospy.init_node('line_following_node', anonymous=True)
    main()

# Taken from github & edited by Muhammad Hafiz Haikal (hafeez.haikal) ~instagram hehe
