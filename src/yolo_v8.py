#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
from ultralytics import data
# from ultralytics.data import *
from time import time

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_msgs.msg import String
from autobot.msg import BoundingBox
from autobot.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError

temp = Image()
bridge = CvBridge()

class Yolo_Dect:
    def __init__(self):

        # load parameters
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov8/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')
        self.visualize = rospy.get_param('~visualize', 'True')

        # which device will be used
        if (rospy.get_param('/use_cpu', 'false')):
            self.device = 'cpu'
        else:
            self.device = 'cuda'

        self.model = YOLO(weight_path)
        self.model.fuse()

        self.model.conf = conf
        self.color_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # output publishers
        self.position_pub = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher(
            '/yolov8/detection_image',  Image, queue_size=1)

        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        temp = image

        self.cv_bridge = bridge.imgmsg_to_cv2(image,desired_encoding="bgr8")
        # cv2.imshow('cropped', cv_bridge)
        # cv2.waitKey(1)

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image, show=False, conf=0.3, verbose=False)

        self.dectshow(results, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, results, height, width):

        self.frame = results[0].plot()
        # print(str(results[0].speed['inference']/1000))
        fps = 1000.0/ results[0].speed['inference']
        # cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        i = 0
        boundingBoxMax = 0
        boundingBox = BoundingBox()
        boundingBox.xmin = np.int64(results[0].boxes.xyxy[0][0].item())
        boundingBox.ymin = np.int64(results[0].boxes.xyxy[0][1].item())
        boundingBox.xmax = np.int64(results[0].boxes.xyxy[0][2].item())
        boundingBox.ymax = np.int64(results[0].boxes.xyxy[0][3].item())
        #boundingBox.Class = results[0].names[result.cls.item()]
        area = (boundingBox.xmax - boundingBox.xmin) * (boundingBox.ymax - boundingBox.ymin)
        areaMax = area
        for result in results[0].boxes:
            # boundingBox = BoundingBox()
            boundingBox.xmin = np.int64(result.xyxy[0][0].item())
            boundingBox.ymin = np.int64(result.xyxy[0][1].item())
            boundingBox.xmax = np.int64(result.xyxy[0][2].item())
            boundingBox.ymax = np.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            area = (boundingBox.xmax - boundingBox.xmin) * (boundingBox.ymax - boundingBox.ymin) 
            if area > areaMax:
                areaMax = area
                boundingBoxMax = i
            else:
                i = i + 1
            self.boundingBoxes.bounding_boxes.append(boundingBox)
        # print(areaMax)
        self.area_pub = rospy.Publisher('/yolov8/area', String, queue_size=1)
        area_data = str(areaMax)
        self.area_pub.publish(area_data)
        self.classify(self.boundingBoxes.bounding_boxes[boundingBoxMax])
        self.position_pub.publish(self.boundingBoxes)
        self.publish_image(self.frame, height, width)

        if self.visualize :
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)

    def classify(self, boundingBox):
        cropped = self.cv_bridge[boundingBox.ymin:boundingBox.ymax, boundingBox.xmin:boundingBox.xmax]
        classify_path = '/home/htqkhanh/catkin_ws/src/autobot/weights/best_classify_new.pt'
        model2 = YOLO(classify_path)
        results2 = model2(cropped, show=False, conf=0.3, verbose=False)
        for result in results2:
            probs = result.probs
            class_index = probs.top1
            class_name = result.names[class_index]
            score = float(probs.top1conf.cpu().numpy())
            # area = (boundingBox.xmax - boundingBox.xmin) * (boundingBox.ymax - boundingBox.ymin)
        # print(class_name)
        sign_data = String()
        # sign_data = str(class_name) + " " + str(area)
        # print(class_name)
        sign_data = str(class_name)
        self.sign_pub = rospy.Publisher('/yolov8/sign', String, queue_size=1)
        self.sign_pub.publish(sign_data)

def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":
    main()