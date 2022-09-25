#! /usr/bin/python
import os
import numpy as np
import cv2
import subprocess
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
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c contrast=32')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c saturation=55')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c hue=0')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c white_balance_temperature_auto=0') # manual mode
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c gamma=100')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c gain=0')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c power_line_frequency=1')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c white_balance_temperature=0')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c sharpness=2')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c backlight_compensation=1')
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c exposure_auto=1') # manual mode
        cmd.append('v4l2-ctl -d /dev/video' + video_id + ' -c exposure_absolute=179')
        
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
        pub_cam = rospy.Publisher("stream", Image, queue_size=1)
        r       = rospy.Rate(30)
        bridge  = CvBridge()
        img     = Image()

        # run publish loop
        while not rospy.is_shutdown():
            ret, frame      = cap.read()
            undistort_image = frame
            undistort_image = cv2.undistort(frame, camera_mat, dist_coef)

            # resize
            # frame           = self.resize(frame)
            undistort_image = self.resize(undistort_image, width_height=(64, 64))
            # undistort_image = self.resize(undistort_image)
            # # add center line
            # frame           = self.add_center_line(frame)
            # undistort_image = self.add_center_line(undistort_image)

            # print(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))

            # pub_cam.publish(bridge.cv2_to_imgmsg(undistort_image, encoding="bgr8"))
            pub_cam.publish(ros_numpy.msgify(Image, undistort_image, encoding='bgr8'))
            
            
            r.sleep()

        cap.release()
        cv2.destroyAllWindows()


    def resize(self, img, width_height=(128, 128)):
        h  = img.shape[1]
        w  = img.shape[0]
        hc = int(h*0.5)

        wc = int(w*0.5)
        img = img[:, hc-wc:hc+wc]

        img = cv2.resize(img , width_height)
        return img


    def add_center_line(self, img): 
        img[int(img.shape[0]/2.0 + 0.5)]    = [255,191,0]
        img[:, int(img.shape[1]/2.0 + 0.5)] = [255,191,0]
        return img


if __name__ == "__main__":

    cs = CameraNode()
    cs.publish_loop()
