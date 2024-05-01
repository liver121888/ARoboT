#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
from arobot.msg import bbox_info,bbox_array
import json


class block2pose:
    def __init__(self):
        self.publisher = rospy.Publisher("/bbox_info", bbox_array, queue_size = 10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            mask_yellow, mask_red, mask_blue = get_color_mask(frame)
            yellow_detection = cv2.bitwise_and(frame, frame, mask=mask_yellow)
            red_detection = cv2.bitwise_and(frame, frame, mask=mask_red)
            blue_detection = cv2.bitwise_and(frame, frame, mask=mask_blue)
            yellow_bboxs = get_bbox(mask_yellow, 'yellow')
            red_bboxs = get_bbox(mask_red, 'red')
            blue_bboxs = get_bbox(mask_blue, 'blue')    
            bboxs = yellow_bboxs + blue_bboxs
            bboxs_msg = bbox_array()
            bboxs_msg.array = bboxs
        except CvBridgeError as e:
            print(e)

        self.publisher.publish(bboxs_msg)

def get_color_mask(frame):
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Yellow color range
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Red color range (HSV wraps around, so there are two ranges)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Blue color range
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # mask format: (W, H)        # (W, H) instead of (H, W) because the format of opencv
    return mask_yellow, mask_red, mask_blue

def get_bbox(mask, color, threshold=20):
    ctrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bboxs = []
    for ctr in ctrs:
        rbox = cv2.minAreaRect(ctr)
        # print(rbox)
        if rbox[1][0] < threshold or rbox[1][1] < threshold:
            continue
        pts = cv2.boxPoints(rbox).astype(np.int32)
        x_max, y_max = pts.max(0)
        x_min, y_min = pts.min(0)
        cx = (x_max + x_min) / 2
        cy = (y_max + y_min) / 2
        info_msg = bbox_info()
        info_msg.color = color
        info_msg.point = [i for i in pts[0]]
        info_msg.center = [cx, cy]
        info_msg.angle = rbox[2]
        # info_dict = {'color': color, 'point': [i for i in pts[0]], 'center': [cx, cy], 'angle': rbox[2]}
        bboxs.append(info_msg)
    # return len(n_box) * info_dict
    # info_dict ->
    # { 'color': should equal to the function input param -> color,
    #   'point': topleft corner point' coordination,
    #   'centor': coordinates for box centers (x_coord, y_coord),
    #   'angle': angle orientation for the boxes (0 ~ 90) }
    return bboxs

def draw_bbox(frame, boxes_list, color_dict={'red':(0,0,255), 'blue':(255,0,0), 'yellow':(0,155,155)}):
    img = frame

    boxes = []
    for box_info in boxes_list:
        cv2.drawContours(img, [box_info['points']], -1, color_dict[box_info['color']], 2, cv2.LINE_AA)

    return img

def main():
    rospy.init_node('block2pose', anonymous=True)
    b2p = block2pose()
    rospy.Subscriber("/camera/color/image_raw", Image, b2p.image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()