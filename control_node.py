#!/usr/bin/env python
# -*- coding: utf-8 -*-
# control_node.py

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import zmq
import json
import threading
import time

# 全局变量，用于存储当前无人机状态
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

class ZeroMQSubscriber(threading.Thread):
    def __init__(self, zmq_sub_address, callback):
        threading.Thread.__init__(self)
        self.zmq_sub_address = zmq_sub_address
        self.callback = callback
        self.daemon = True  # 允许主线程退出时，子线程自动退出

        # 初始化ZeroMQ上下文和SUB套接字
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.zmq_sub_address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, u'')  # 订阅所有消息
        rospy.loginfo("ZeroMQ SUB socket connected to {}".format(self.zmq_sub_address))

    def run(self):
        while not rospy.is_shutdown():
            try:
                message = self.socket.recv()
                data = json.loads(message)
                rospy.loginfo("Received target coordinates via ZeroMQ: x={}, y={}, z={}".format(
                    data['x'], data['y'], data['z']))
                self.callback(data)
            except Exception as e:
                rospy.logerr("Error receiving target coordinates: {}".format(e))

class ControlNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('control_node', anonymous=True)

        # 订阅无人机当前状态
        rospy.Subscriber('mavros/state', State, state_cb)

        # 发布位姿信息到mavros/setpoint_position/local
        self.pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # 等待MAVROS服务
        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/cmd/arming')
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        rospy.loginfo("MAVROS services are now available.")

        # 设置ZeroMQ订阅地址
        zmq_sub_address = "tcp://localhost:5556"  # 根据实际情况修改
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0
        self.target_pose.pose.position.y = 0
        self.target_pose.pose.position.z = 1.5  # 设定初始高度

        # 设置PoseStamped的header信息
        self.target_pose.header.frame_id = "map"  # 根据实际情况修改
        self.target_pose.header.stamp = rospy.Time.now()

        # 初始化ZeroMQ订阅者线程
        self.zmq_subscriber = ZeroMQSubscriber(zmq_sub_address, self.update_target_pose)
        self.zmq_subscriber.start()

        # 发布频率（必须高于2Hz）
        self.rate = rospy.Rate(20.0)

        # 获取图像分辨率
        self.image_width = rospy.get_param("~image_width", 640)  # 默认640，需根据实际情况修改
        self.image_height = rospy.get_param("~image_height", 480)  # 默认480，需根据实际情况修改
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2

        # 中心阈值（根据需要调整）
        self.center_threshold_x = 60  # 像素
        self.center_threshold_y = 60  # 像素

        # 比例控制器参数
        self.Kp_x = 100  # 控制前进/后退速度
        self.Kp_y = 0.001  # 控制左右移动速度
        self.Kp_yaw = 0.002  # 控制偏航角速度
        self.max_velocity = 0.5  # 最大速度增量，单位：m/s

        # 当前目标误差
        self.error_x = 0
        self.error_y = 0

        # 控制状态
        self.centered = False

        # 等待FCU连接
        while not rospy.is_shutdown() and not current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            self.pose_pub.publish(self.target_pose)  # 持续发布以维持OFFBOARD模式
            self.rate.sleep()

        rospy.loginfo("FCU connected.")

        # 发布一些位姿点以稳定通信
        for i in range(100):
            self.pose_pub.publish(self.target_pose)
            self.rate.sleep()

        # 设置模式为OFFBOARD并解锁无人机
        last_request = rospy.Time.now()

        # 请求进入OFFBOARD模式和解锁
        while not rospy.is_shutdown() and (current_state.mode != "OFFBOARD" or not current_state.armed):
            current_time = rospy.Time.now()
            if current_time - last_request > rospy.Duration(5.0):
                if current_state.mode != "OFFBOARD":
                    try:
                        # 请求进入OFFBOARD模式
                        res = self.set_mode(custom_mode="OFFBOARD")
                        if res.mode_sent:
                            rospy.loginfo("Offboard enabled")
                        else:
                            rospy.logerr("Failed to set OFFBOARD mode")
                    except rospy.ServiceException as e:
                        rospy.logerr("SetMode service call failed: {}".format(e))
                else:
                    try:
                        # 请求解锁无人机
                        res = self.arming_client(True)
                        if res.success:
                            rospy.loginfo("Vehicle armed")
                        else:
                            rospy.logerr("Failed to arm vehicle")
                    except rospy.ServiceException as e:
                        rospy.logerr("Arming service call failed: {}".format(e))
                last_request = current_time
            self.pose_pub.publish(self.target_pose)
            self.rate.sleep()

        rospy.loginfo("OFFBOARD mode enabled and vehicle armed.")

    def update_target_pose(self, data):
        """
        更新目标位姿信息，根据目标在图像中的位置调整无人机的移动方向
        """
        # 计算目标在图像中的偏差
        self.error_x = data['x'] - self.center_x  # 水平偏差
        self.error_y = data['y'] - self.center_y  # 垂直偏差

    def run(self):
        """
        主循环，持续发布目标位姿
        """
        while not rospy.is_shutdown():
            # 判断目标是否在中心范围内
            if abs(self.error_x) > self.center_threshold_x or abs(self.error_y) > self.center_threshold_y:
                self.centered = False
            else:
                self.centered = True

            if not self.centered:
                # 调整Y坐标使目标居中
                # 根据误差调整Y方向
                # 目标在图像右侧（error_x > 0） -> 无人机向右移动（减少 y）
                # 目标在图像左侧（error_x < 0） -> 无人机向左移动（增加 y）
                delta_y = self.Kp_y * (-self.error_x)
                delta_y = max(min(delta_y, self.max_velocity), -self.max_velocity)
                delta_x = 0.2  # 不调整前后移动
                rospy.loginfo("Adjusting Y to center the target: delta_x=0, delta_y={}".format(delta_y))
            else:
                # 目标已居中，向前移动
                # 增加X坐标使无人机向前移动
                delta_x = self.Kp_x  # 可以根据需要调整为一个固定值或比例值
                delta_x = max(min(delta_x, self.max_velocity), -self.max_velocity)
                delta_y = 0  # 不调整左右移动
                rospy.loginfo("Target centered. Moving forward: delta_x={}, delta_y=0".format(delta_x))

            # 获取当前位姿
            try:
                current_pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=1)
            except rospy.ROSException:
                rospy.logwarn("Failed to get current pose.")
                current_pose = self.target_pose  # 使用最后的目标位姿

            # 更新目标位姿基于当前位姿和增量
            new_x = current_pose.pose.position.x + delta_x
            new_y = current_pose.pose.position.y + delta_y
            new_z = 1.5  # 保持固定高度

            # 更新目标位姿
            self.target_pose.pose.position.x = new_x
            self.target_pose.pose.position.y = new_y
            self.target_pose.pose.position.z = new_z

            # 更新header的时间戳
            self.target_pose.header.stamp = rospy.Time.now()

            # 发布新的目标位姿
            self.pose_pub.publish(self.target_pose)

            # 记录日志（可选）
            rospy.loginfo("Publishing target_pose: x={:.3f}, y={:.3f}, z={:.3f}".format(
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z
            ))

            # 保持发布频率
            self.rate.sleep()

def main():
    try:
        control_node = ControlNode()
        control_node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
