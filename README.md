# eBNN on NodeMCU with micro-ROS
This system integrates ROS 2 and containerized deployment with embedded eBNN inference and PCA-based anomaly detection to monitor abnormal inputs and behaviors in micro-ROS-enabled devices.
## System Architecture
![system_topology](./images/system_topology.png)
## Project Structure
```
eBNN-microROS
├── include
│   ├── ebnn.h
│   └── simple_mnist.h
├── platformio.ini
└── src
    ├── client
    │   └── main.cpp
    └── host
        ├── agent
        │   └── run_agent.sh
        ├── monitor
        │   ├── Dockerfile
        │   ├── ebnn_monitor
        │   │   ├── ebnn_monitor
        │   │   │   ├── __init__.py
        │   │   │   ├── anomaly_detector.py
        │   │   │   └── ebnn_monitor.py
        │   │   ├── package.xml
        │   │   ├── resource
        │   │   │   └── ebnn_monitor
        │   │   ├── setup.cfg
        │   │   └── setup.py
        │   ├── requirements.txt
        │   ├── run_monitor.sh
        │   └── tools
        │       └── train_pca.py
        └── publisher
            ├── Dockerfile
            ├── mnist_publisher
            │   ├── mnist_publisher
            │   │   ├── __init__.py
            │   │   └── mnist_publisher.py
            │   ├── package.xml
            │   ├── resource
            │   │   └── mnist_publisher
            │   ├── setup.cfg
            │   └── setup.py
            └── run_publisher.sh

```
## Technology Stack
### Operating System & Environment
- **Ubuntu 22.04**
- **Docker**
    - Containerized deployment for ROS 2 nodes, agent, model training, and monitoring services
    - Ensures reproducibility and environment consistency across systems
### Robotics Middleware
- **ROS 2 Humble**
    - Node-based distributed architecture
    - Asynchronous, topic-based communication
    - Configurable QoS policies (Reliability, History, Depth
- **micro-ROS**
    - Deploy an embedded Binary Neural Network (eBNN) on microcontrollers
    - Publish inference results to the ROS 2 network
### Programming Language
- **Python 3.10+**
    - ROS 2 node implementation using `rclpy`
    - Data preprocessing, inference monitoring, and anomaly detection
- **C/C++**
    - Embedded-side implementation for micro-ROS and eBNN deployment
### Deep Learning Models
- **eBNN (Embedded Binary Neural Network)**
    - Deployed on embedded/microcontroller devices
    - Ultra-low memory footprint and energy-efficient inference
### Anomaly Detection
- **PCA (Principal Component Analysis)**
    - Learns a low-dimensional subspace representing normal MNIST data
    - Uses reconstruction error as the anomaly score
- **Scikit-learn**
    - PCA model training and inference
- **Joblib**
    - Model serialization and cross-container loading
### Datasets
- **MNIST Dataset**
    - Normal samples from official TensorFlow / PyTorch MNIST datasets
    - Anomalous samples generated via:
        - Random noise matrices
        - Corrupted or partially occluded images
## Prerequisites
### Hardware Requirements
|Item|Quantity|Notes|
|---|---|---|
|ESP32 board (NodeMCU-32S)|1|
|USB cables|1|One end must be micro-B|
|Development machine|1|Ubuntu 22.04 LTS recommended|
### Software Requirements
|Components|Purpose|
|---|---|
|VScode with PlatformIO extension|IDE|
|Docker|Start publisher and subscriber services|

## Quick Start Guide
1. Clone the repo and open it with VScode (PlatformIO extension required).

    ```bash
    git clone https://github.com/Steven0811/eBNN-microROS.git
    cd eBNN-microROS
    ```
2. Wait until PlatformIO successfully initializing the project.
3. Connect ESP32 and your development machine with USB cable.
4. Run and upload the code to ESP32.
    ```bash
    # in PlatformIO CLI
    pio run -t upload --upload-port <your ESP32 port>
    ```
5. Start agent.
    ```bash
    # Terminal 1: Flash firmware and start agent
    cd eBNN-microROS/src/host/agent
    bash run_agent.sh
    ```
6. Start ebnn monitor.
    ```bash
    # Terminal 2: Run subscriber
    cd eBNN-microROS/src/host/monitor
    bash run_monitor.sh
    ```
7. Start publisher.
    ```bash
    # Terminal 3: Run publisher
    cd eBNN-microROS/src/host/publisher
    bash run_publisher.sh
    ```
## License
This project is licensed under the MIT License.

This project uses third-party components:
- eBNN (MIT)
- micro_ros_platformio (Apache-2.0)
## Reference
- eBNN: https://github.com/kunglab/ebnn
- micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
- micro-ROS: https://micro.ros.org/
- ROS2: https://docs.ros.org/
- PlatfromIO: https://platformio.org/