#! /usr/bin/python3
import os
import numpy as np
import cv2
import subprocess

from numpy.lib.type_check import imag
import rospy, ros_numpy, rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class CameraNode:
    def __init__(self, width=1280, height=720):
        self.width  = width
        self.height = height

        # camera parameter setting
        cmd = list()
        video_id = "0"
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c brightness=0')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c contrast=32') # defo
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c saturation=55')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c hue=0') # defo
        # cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c hue=128')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c white_balance_temperature_auto=0') # manual mode
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c gamma=100')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c gain=0') # 使えなくなっている？（2025/01/22 by tomoya-y）
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c power_line_frequency=1')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c white_balance_temperature=0')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c sharpness=2')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c backlight_compensation=1')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c exposure_auto=1') # manual mode
        # cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c exposure_absolute=179') # defo
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c exposure_absolute=300')
        # cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c exposure_absolute=70')

        for cmd_i in cmd:
            subprocess.check_output(cmd_i, shell=True)



    def publish_loop(self):
        # load calibration data
        rospack = rospkg.RosPack()
        path = rospack.get_path('ros_camera') + "/config/calib_opencv/"
        camera_mat = np.loadtxt(path+'K.csv', delimiter=',')
        dist_coef  = np.loadtxt(path+'d.csv', delimiter=',')

        # instance of vedeo capture
        cap = cv2.VideoCapture(-1)
        cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # ros setting
        rospy.init_node("image_raw_node", disable_signals=True)
        pub_cam_origin = rospy.Publisher("image_origin", Image, queue_size=1)
        pub_cam_resize = rospy.Publisher("image_resize", Image, queue_size=1)

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, frame      = cap.read()
            # print("cap.red() shape = ", frame.shape)
            undistort_image        = frame
            undistort_image        = cv2.undistort(frame, camera_mat, dist_coef)
            img_origin, img_resize = self.resize(undistort_image, width_height=(64, 64))

            # print("img_origin shape = ", img_origin.shape)
            # print("img_resize shape = ", img_resize.shape)

            pub_cam_origin.publish(ros_numpy.msgify(Image, img_origin, encoding='bgr8'))
            pub_cam_resize.publish(ros_numpy.msgify(Image, img_resize, encoding='bgr8'))

            r.sleep()

        cap.release()
        cv2.destroyAllWindows()


    def convert_specific_color(self, image):
        # print(image.max())
        # print(image.min())
        bgrLower = np.array([130]*3)
        bgrUpper = np.array([255]*3)
        mask     = cv2.inRange(image, bgrLower, bgrUpper)
        image[mask>0]=(0,0,0)
        # result   = cv2.bitwise_and(image, image, mask=img_mask)
        return image


    def resize(self, img, width_height=(128, 128)):
        h          = img.shape[1]
        w          = img.shape[0]
        # hc         = int(h*0.5)
        hc         = int(h*0.51) # 中心にバルブを持ってくるため
        wc         = int(w*0.5)
        img_origin = img[:, hc-wc:hc+wc]

        img_origin = img_origin[30:, 30:] # 中心にバルブを持ってくるため
        img_resize = cv2.resize(img_origin , width_height)
        return img_origin, img_resize


    def add_center_line(self, img):
        img[int(img.shape[0]/2.0 + 0.5)]    = [255,191,0]
        img[:, int(img.shape[1]/2.0 + 0.5)] = [255,191,0]
        return img


if __name__ == "__main__":

    cs = CameraNode()
    cs.publish_loop()
