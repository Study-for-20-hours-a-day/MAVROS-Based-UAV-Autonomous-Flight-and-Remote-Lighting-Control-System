# **Autonomous UAV Flight and Remote Lighting Control System**
【无人机演示】https://www.bilibili.com/video/BV1wDwQemEV3?vd_source=003215dcbf37b2c6060f4782a5c7afa5

## **Project Overview**
This project implements a **multi-functional UAV system** that combines **autonomous flight control**, **real-time lighting optimization**, and **adaptive posture adjustment**. The system is designed to optimize **low-altitude detection** by dynamically adjusting the onboard lighting and the UAV’s posture based on real-time environmental conditions. The core technology is based on **Continuous-Time Neural Networks (CTNN)**, which allows the UAV to continuously adapt to changes in lighting and flight posture, ensuring optimal detection performance.

---

## **Key Features**
- **Autonomous UAV Flight Control**: The UAV autonomously navigates a predefined flight path using **MAVROS** for offboard control.
- **Real-time Lighting Adjustment**: The onboard lighting is dynamically adjusted based on the lighting conditions and real-time image analysis, powered by a **CTNN-based model**.
- **Posture Adjustment**: The UAV adjusts its posture (pitch, roll, yaw) in real-time to optimize sensor performance and enhance image quality during flight.
- **Embedded System Deployment**: The system is designed for deployment on **Jetson Orin Nano**, with real-time image processing and control.

---

## **Technical Approach**
### **1. Autonomous Flight Control**  
- The UAV uses **MAVROS** for **offboard control**, integrating it with **PX4** for flight state management and navigation.  
- The system monitors UAV states, adjusts flight position, and controls the altitude with real-time updates from the onboard **IMU** and **GPS**.

### **2. CTNN-based Adaptive Lighting Optimization**
- **CTNN (Continuous-Time Neural Networks)** is used to model the relationship between lighting intensity and image quality.
- The UAV adjusts the **LED lighting intensity** through **PWM** based on the output of the CTNN model.
- The model uses **real-time image inputs** and **flight posture data** to predict the optimal lighting intensity that improves target detection accuracy.

### **3. Real-time Posture Adjustment**
- **Posture optimization** is handled by the same CTNN, which uses the UAV’s current posture (pitch, roll, yaw) and image quality as inputs.
- The system computes the best **posture configuration** to maximize the effectiveness of the lighting and sensor data collection, ensuring optimal target detection.

### **4. Real-time Image Processing and Control**
- **ZeroMQ** is used for **real-time image transmission** from the UAV’s camera to the ground station.
- A remote control system, using **XML-RPC**, allows for dynamic adjustment of lighting parameters via a web-based interface.

---

## **System Components**
- **Jetson Orin Nano**: Embedded platform for running the **CTNN model**, controlling **PWM**, and handling **image processing**.
- **MAVROS & PX4**: Software stack for flight control and navigation.
- **CTNN Model**: A neural network used to predict the optimal lighting intensity and UAV posture adjustments.
- **LED Lighting**: RGB LEDs for adjustable lighting, controlled via **GPIO** pins on the Jetson Orin Nano.
- **ZeroMQ**: Real-time communication protocol for transmitting images to the ground station.
- **XML-RPC**: For remote control of lighting adjustments via the ground station.

---

## **Installation**

### **1. Prerequisites**
Ensure you have the following installed:
- **ROS Noetic** (or a compatible version)
- **MAVROS** package
- **Jetson Orin Nano** setup with **Ubuntu 20.04** and **Jetson.GPIO**
- **ZeroMQ** library for Python
- **PyTorch** for running the CTNN model

### **2. Clone the Repository**
```bash
git clone https://github.com/yourusername/Autonomous-UAV-Control.git
cd Autonomous-UAV-Control
```

### **3. Install Dependencies**
```bash
# Install ROS dependencies
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# Install ZeroMQ
pip install pyzmq

# Install PyTorch
pip install torch torchvision
```

### **4. Set Up MAVROS**
Ensure that MAVROS is configured correctly for communication with your UAV's flight controller (PX4).
```bash
roslaunch mavros px4.launch
```

### **5. Run the System**
Start the UAV control and monitoring system:
```bash
# On the UAV machine (Jetson Orin Nano)
roslaunch autonomous_uav_control start_flight_control.launch

# On the ground station
python run_ground_station.py
```

---

## **Usage**
- **Flight Control**: Once the system is running, the UAV will autonomously take off, follow the predefined flight path, and adjust its posture to optimize target detection.
- **Lighting Adjustment**: The UAV dynamically adjusts its onboard lighting based on the current environmental conditions, with the lighting intensity controlled by **PWM**.
- **Real-time Monitoring**: The system allows for real-time monitoring of the UAV’s flight state, posture, and lighting through a **ground control interface**.

---

## **Future Work**
- **Multi-source Lighting Optimization**: Future enhancements could include **RGB dynamic lighting control** to adjust the color temperature for various environments.
- **Enhanced Posture Adjustment**: Integration with additional sensors like **IMUs** and **LIDAR** for more precise posture optimization.
- **Real-time Feedback Loop**: Implement a more advanced **reinforcement learning** approach for real-time feedback-based optimization of lighting and posture.

---

## **Contributors**
- **Your Name** – Lead Developer
- **Collaborator Name** – UAV Flight Control Specialist
- **Collaborator Name** – Computer Vision and Model Optimization Specialist

---

## **License**
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---

Feel free to adapt the README as necessary based on your project specifics!
