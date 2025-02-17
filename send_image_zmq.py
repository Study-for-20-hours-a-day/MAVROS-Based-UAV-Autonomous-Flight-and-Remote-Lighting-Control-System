#!/usr/bin/env python
# -*- coding: utf-8 -*-
# send_image_zmq.py

import rospy
from sensor_msgs.msg import Image
import ros_numpy
import zmq
import base64
import cv2

def image_callback(msg):
    # 将 ROS Image 消息转换为 numpy 数组
    np_image = ros_numpy.numpify(msg)
    
    # 检查图像通道数并转换颜色空间（如果需要）
    if len(np_image.shape) == 3 and np_image.shape[2] == 3:
        np_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)
    
    # 将图像编码为 JPEG 格式
    success, buffer = cv2.imencode('.jpg', np_image)
    if success:
        jpg_as_text = base64.b64encode(buffer)
        # 通过 ZeroMQ 发送图像数据
        socket.send(jpg_as_text)
        rospy.loginfo("Sent image via ZeroMQ")
    else:
        rospy.logwarn("图像编码失败")

def send_image_zmq():
    rospy.init_node('send_image_zmq', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    # 初始化 ZeroMQ 上下文和套接字
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")  # 绑定到 5555 端口用于发布图像
    rospy.loginfo("ZeroMQ PUB socket bound to tcp://*:5555")
    send_image_zmq()

