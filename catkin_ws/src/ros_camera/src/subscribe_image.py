import sys
import os
import time
import numpy as np
import cv2
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Int32
import pickle



class SubscribeImage():
    def __init__(self):

        rospy.init_node('image_test', anonymous=True)
        rospy.Subscriber('/image_raw', Image, self.callback_img)
    

    def callback_img(self, img):
        # print(img)
        self.image = ros_numpy.numpify(img)
        # print(self.image)

    def run(self): 

        for i in range(300): 
            cv2.imshow("fff", self.image)
            cv2.waitKey(100) # shape is OK!


if __name__ == '__main__':
    si = SubscribeImage()

    rospy.sleep(2)

    si.run()