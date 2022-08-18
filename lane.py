#!/usr/bin/env python3

import matplotlib.pylab as plt
import cv2
import cv_bridge
import numpy as np
import rospy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/catvehicle/camera_front/image_raw_front',
                                          Image, self.region_of_interest)
        self.twist = Twist()

    def region_of_interest(self, vertices):
        mask = np.zeros_like(self)
        # channel_count = img.shape[2]
        match_mask_color = (255,) * channel_count
        cv2.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv2.bitwise_and(self, mask)
        return masked_image

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        print(image.shape)
        height = image.shape[0]
        width = image.shape[1]

        region_of_interest_vertices = [
            (0, height),
            (width / 2, height / 2)
        ]
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        canny_image = cv2.Canny(gray_image, 100, 200)
        cropped_image = region_of_interest(canny_image,
                                           np.array([region_of_interest_vertices], np.int32), )
        plt.imshow(cropped_image)
        plt.show()
