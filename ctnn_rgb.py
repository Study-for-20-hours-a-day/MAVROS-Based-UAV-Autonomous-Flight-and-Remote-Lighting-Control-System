#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ctnn_rgb.py
import os
import time
import base64
import cv2
import zmq
import numpy as np
from PIL import Image
import xmlrpc.client
import re

import torch
import torch.nn as nn
from torchvision import transforms

# -------------------------------------
# 1. 零MQ订阅图像
# -------------------------------------
class ZMQImageSubscriber:
    """
    通过ZeroMQ订阅远程发送的图像数据，并在本地提供获取图像的接口。
    """
    def __init__(self, zmq_sub_addr="tcp://10.42.0.138:5555"):
        """
        :param zmq_sub_addr: 发送端的ZeroMQ PUB地址，例如 "tcp://<远程IP>:5555"
        """
        self.zmq_sub_addr = zmq_sub_addr
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.zmq_sub_addr)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息

    def receive_image(self, timeout_ms=5000):
        """
        接收一次图像数据，默认5秒超时。返回OpenCV格式的图像（BGR格式）。
        若超时或解析失败，返回 None。
        """
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)

        socks = dict(poller.poll(timeout_ms))
        if socks.get(self.socket) == zmq.POLLIN:
            jpg_as_text = self.socket.recv()
            # 解码Base64
            try:
                jpg_buffer = base64.b64decode(jpg_as_text)
                # 转成numpy再解码
                np_arr = np.frombuffer(jpg_buffer, dtype=np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                return img
            except Exception as e:
                print(f"[WARN] 图像解码失败: {e}")
                return None
        else:
            print(f"[WARN] 超时未接收到图像消息")
            return None

# -------------------------------------
# 2. 定义CTNN模型
# -------------------------------------
class CFC(nn.Module):
    def __init__(self, input_size, hidden_size):
        super(CFC, self).__init__()
        self.hidden_size = hidden_size
        self.W = nn.Linear(input_size, hidden_size)
        self.U = nn.Linear(hidden_size, hidden_size)
        self.V = nn.Linear(hidden_size, 1)

    def forward(self, x):
        # x: (batch_size, seq_len, input_size)
        h = torch.zeros(x.size(0), self.hidden_size).to(x.device)
        for t in range(x.size(1)):
            h = torch.tanh(self.W(x[:, t, :]) + self.U(h))
        out = self.V(h)
        return out.squeeze()

class BrightnessCFC(nn.Module):
    def __init__(self):
        super(BrightnessCFC, self).__init__()
        self.feature_extractor = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, padding=1),  # HSV通道数为3
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1))  # 输出尺寸: 1x1
        )
        self.cfc = CFC(input_size=32 + 1, hidden_size=64)

    def forward(self, images, adjusted_vs):
        batch_size, seq_len, C, H, W = images.size()
        images = images.view(-1, C, H, W)
        features = self.feature_extractor(images)  # (batch_size * seq_len, 32, 1, 1)
        features = features.view(batch_size, seq_len, -1)  # (batch_size, seq_len, 32)
        adjusted_vs = adjusted_vs.unsqueeze(-1)  # (batch_size, seq_len, 1)
        inputs = torch.cat((features, adjusted_vs), dim=2)  # (batch_size, seq_len, 33)
        output = self.cfc(inputs)  # (batch_size)
        return output

# -------------------------------------
# 3. HSV转换和亮度计算函数
# -------------------------------------
def load_image_and_compute_v_from_bgr(bgr_image):
    """
    输入：BGR格式的图像(numpy array)
    输出：HSV张量（归一化到[0,1]），以及V通道平均值（归一化到[0,1]）
    """
    # 统一resize到(256,256)以便后续处理
    bgr_image = cv2.resize(bgr_image, (256, 256))
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    v_channel = hsv_image[:, :, 2]
    average_v = np.mean(v_channel)
    hsv_tensor = torch.from_numpy(hsv_image).permute(2, 0, 1).float() / 255.0
    return hsv_tensor, average_v / 255.0

# -------------------------------------
# 4. 主流程
# -------------------------------------
def main():
    """
    - 远程控制无人机三路灯光(R/G/B)的PWM占空比
      顺序：先全关 -> 分别单独亮(R/G/B) 30,60,100 -> 全部亮(30,60,100)
    - 每次等待3秒后，通过ZeroMQ接收图像并保存
    - 所有采集完成后，运行CTNN推理 -> 找到最佳亮度 -> 远程调光到此(R,G,B)
    - **新增**：在完成最优调光后，再等待3秒并保存当前图像
    - 最后删除predict文件夹内的所有图片(排除最终图像)
    """

    # 4.1 准备 ZeroMQ 订阅
    zmq_sub_addr = "tcp://10.42.0.138:5555"  # 根据实际地址修改
    zmq_subscriber = ZMQImageSubscriber(zmq_sub_addr=zmq_sub_addr)
    print(f"[INFO] 已连接 ZeroMQ: {zmq_sub_addr}")

    # 4.2 准备 XML-RPC 远程灯光控制
    SERVER_URL = "http://10.42.0.1:8000/"  # 机载电脑上的 RPC 服务
    proxy = xmlrpc.client.ServerProxy(SERVER_URL)
    print(f"[INFO] 已连接 XML-RPC: {SERVER_URL}")

    # 4.3 采集多张图像：三路灯占空比序列
    # 三路灯（RGB）对应：GPIO11, GPIO12, GPIO13
    duty_sequences = [
        (0, 0, 0),         # 全部关
        (30, 0, 0), (60, 0, 0), (100, 0, 0),
        (0, 30, 0), (0, 60, 0), (0, 100, 0),
        (0, 0, 30), (0, 0, 60), (0, 0, 100),
        (30, 30, 30), (60, 60, 60), (100, 100, 100),
    ]

    predict_dir = "predict"
    if not os.path.exists(predict_dir):
        os.makedirs(predict_dir)

    # 依次设置并采集图像
    for (r_duty, g_duty, b_duty) in duty_sequences:
        print(f"\n[STEP] 设置占空比: R={r_duty}% G={g_duty}% B={b_duty}%")
        # 分别设置 GPIO11, GPIO12, GPIO13
        response_r = proxy.set_pwm(11, 100, r_duty)
        response_g = proxy.set_pwm(12, 100, g_duty)
        response_b = proxy.set_pwm(13, 100, b_duty)
        print(f"  -> pin11={r_duty}% resp={response_r}")
        print(f"  -> pin12={g_duty}% resp={response_g}")
        print(f"  -> pin13={b_duty}% resp={response_b}")

        # 等待3秒，灯光稳定
        time.sleep(3)

        # 接收图像 (带3秒超时)
        img_bgr = zmq_subscriber.receive_image(timeout_ms=3000)
        if img_bgr is None:
            print(f"[WARN] 没能接收图像, 跳过此步")
            continue

        # 保存图像
        save_path = os.path.join(predict_dir, f"brightness_{r_duty}_{g_duty}_{b_duty}.jpg")
        cv2.imwrite(save_path, img_bgr)
        print(f"[INFO] 图像已保存: {save_path}")

    print("[INFO] 所有占空比级别已采集完毕，准备进行 CTNN 推理。")

    # 4.4 加载 CTNN 模型
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = BrightnessCFC().to(device)

    # 假设模型文件为 brightness_cfc_model.pth
    model_path = "brightness_cfc_model.pth"
    if not os.path.exists(model_path):
        print(f"[ERROR] 模型文件 {model_path} 不存在，无法进行推理。")
        return
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    print("[INFO] CTNN 模型加载完成。")

    # 4.5 准备推理数据
    image_list = []
    v_values = []
    image_paths = []

    # 新的正则匹配 (brightness_r_g_b.jpg)
    brightness_pattern = re.compile(r"brightness_(\d+)_(\d+)_(\d+)\.jpg")

    for filename in os.listdir(predict_dir):
        if not (filename.endswith(".jpg") or filename.endswith(".png")):
            continue

        match = brightness_pattern.match(filename)
        if not match:
            # 不符合命名规则的文件跳过
            continue

        # 读取图像
        img_path = os.path.join(predict_dir, filename)
        bgr_img = cv2.imread(img_path)
        if bgr_img is None:
            continue

        hsv_tensor, avg_v = load_image_and_compute_v_from_bgr(bgr_img)
        image_list.append(hsv_tensor)
        v_values.append(avg_v)
        image_paths.append(img_path)

    if len(image_list) == 0:
        print("[ERROR] predict 文件夹内没有符合命名规则的图像，退出。")
        return

    # 打包成张量: (1, num_images, C, H, W)
    images = torch.stack(image_list).unsqueeze(0).to(device)
    adjusted_vs = torch.tensor(v_values, dtype=torch.float32).unsqueeze(0).to(device)

    # 4.6 CTNN 推理
    with torch.no_grad():
        optimal_v = model(images, adjusted_vs).item()
    print(f"[RESULT] 预测的最佳亮度值(归一化): {optimal_v:.4f}")
    print(f"         预测的最佳亮度值(0~255): {optimal_v*255:.2f}")

    # 4.7 找到最接近此亮度的图像 -> 得到该图像对应的 (r, g, b)
    v_values_np = np.array(v_values)
    closest_idx = np.abs(v_values_np - optimal_v).argmin()
    closest_img_path = image_paths[closest_idx]
    print(f"[INFO] 与最佳亮度最接近的图像: {closest_img_path}")

    # 从文件名中提取 (r, g, b)
    match = brightness_pattern.match(os.path.basename(closest_img_path))
    if match:
        closest_r = int(match.group(1))
        closest_g = int(match.group(2))
        closest_b = int(match.group(3))
    else:
        # 万一命名不规范，就粗略地取 round(optimal_v * 100)
        closest_r = closest_g = closest_b = int(round(optimal_v * 100))

    print(f"[INFO] 最佳占空比: R={closest_r}%, G={closest_g}%, B={closest_b}%")

    # 4.8 调整无人机上GPIO到此 (r, g, b)
    for pin, duty in zip([11, 12, 13], [closest_r, closest_g, closest_b]):
        response = proxy.set_pwm(pin, 100, duty)
        print(f"  -> pin {pin}, duty={duty}%, response: {response}")

    print("[INFO] 调整完毕。您可以在无人机上查看灯光效果。")

    # 4.8.1 等待3秒后，再次获取图像并保存
    time.sleep(3)
    final_img = zmq_subscriber.receive_image(timeout_ms=3000)
    if final_img is not None:
        final_img_path = os.path.join(predict_dir, "final_after_optimal.jpg")
        cv2.imwrite(final_img_path, final_img)
        print(f"[INFO] 已保存最终调光后的图像: {final_img_path}")
    else:
        print(f"[WARN] 等待3秒后未能接收图像，无法保存最终调光照片。")

    # 4.9 自动删除 predict 文件夹中的图片
    # 为了保留刚才那张 final_after_optimal.jpg，这里做一下排除
    print("[INFO] 正在清理 predict 文件夹中的图片(排除 final_after_optimal.jpg)...")
    for filename in os.listdir(predict_dir):
        file_path = os.path.join(predict_dir, filename)
        # 跳过最终图像
        if filename == "final_after_optimal.jpg":
            continue
        if os.path.isfile(file_path):
            os.remove(file_path)
    print("[INFO] 其他图片已删除，final_after_optimal.jpg 已保留。")

if __name__ == "__main__":
    main()
