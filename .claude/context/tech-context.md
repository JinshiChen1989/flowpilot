---
created: 2025-08-29T18:15:20Z
last_updated: 2025-08-29T18:15:20Z
version: 1.0
author: Claude Code PM System
---

# Technology Context

## Primary Technology Stack

### Core Framework
- **ROS2** (Robot Operating System 2) - Distributed robotics framework
- **Python 3.10+** - Primary development language
- **PyQt5** (5.15.9) - GUI framework for user interface
- **C++** - Performance-critical components and hardware interfaces

### Development Environment
- **Ubuntu Linux** - Primary development and deployment platform
- **Git** with submodules - Version control and dependency management
- **Ament** - ROS2 build system for Python packages
- **CMake** - Build system for C++ components

## AI/ML Technology Stack

### Model Formats
- **ONNX** - Cross-platform neural network models
- **HEF** - Hailo AI accelerator optimized models
- **RKNN** - Rockchip Neural Processing Unit models

### Computer Vision
- **OpenCV** - Computer vision processing
- **ByteTrack** - Multi-object tracking algorithm
- **YOLO** - Object detection models
- **Stereo Vision** - Depth perception processing

### Hardware Acceleration
- **Hailo AI Accelerator** - Neural network inference acceleration
- **Rockchip NPU** - On-device AI processing
- **GPU Acceleration** - CUDA/OpenCL for vision processing

## Communication & Networking

### Inter-Process Communication
- **ROS2 Topics** - Publish/subscribe messaging
- **ROS2 Services** - Request/response patterns
- **ROS2 Actions** - Long-running task management
- **DDS** (Data Distribution Service) - Underlying middleware

### Hardware Communication
- **CAN Bus** - Vehicle network communication
- **I2C** - Inter-integrated circuit sensor communication (I2C4 bus on RK3588)
- **UART/Serial** - Hardware device interfaces (UART4 @ 115200 bps for microROS)
- **USB** - Camera and peripheral connectivity
- **MicroROS** - Lightweight ROS2 for microcontrollers via UART

### Network Protocols
- **MQTT** - IoT messaging protocol
- **TCP/UDP** - Network communication
- **WebSocket** - Real-time web communication

## Data Processing & Storage

### Data Formats
- **ROS2 Bag** - Data recording and playback
- **MCAP** - Modern container for robotics data
- **JSON/YAML** - Configuration and metadata
- **Protocol Buffers** - Serialization protocol

### Storage Systems
- **SQLite** - Local data persistence
- **Cloud Storage** - AWS S3 integration via boto3
- **Log Management** - Structured logging with rotation

## Sensor & Hardware Integration

### Camera Systems
- **Multi-camera arrays** (6 cameras: front L/R, rear L/R, driver, road)
- **Stereo vision** - Depth perception
- **USB Video Class** - Standardized camera interfaces

### Positioning & Navigation
- **GPS/GNSS** - Global positioning
- **RTK** - Real-time kinematic positioning
- **IMU** - Inertial measurement units
- **Compass** - Magnetic heading sensors

### Environmental Sensors
- **Temperature/Humidity** - Environmental monitoring (SHT40-AD)
- **Light sensors** - Ambient lighting detection (VEML7700)
- **IMU Sensors** - Inertial measurement (ICM-42688)
- **Compass** - Magnetic heading sensors (AK09918)
- **Barometric pressure** - Altitude sensing

## Development Dependencies

### Core Libraries (from bootloader/requirements.txt)
```
PyQt5==5.15.9
numpy==1.26.4
Pillow==10.0.1
pyyaml==6.0.1
psutil>=5.7.0
netifaces>=0.11.0
watchdog>=3.0.0
requests>=2.25.0
cryptography>=3.4.0
boto3>=1.20.0
paho-mqtt>=1.5.0
```

### ROS2 Dependencies
- **rclpy** - Python client library (primary for sensing system)
- **geometry_msgs** - Geometric data structures
- **sensor_msgs** - Standard sensor data messages (IMU, camera, GPS)
- **visualization_msgs** - Visualization data
- **tf2** - Transform libraries
- **cv_bridge** - OpenCV-ROS integration
- **diagnostic_msgs** - System health monitoring
- **smbus** - I2C bus communication library

### Testing Framework
- **pytest** - Python testing framework
- **ament_lint** - ROS2 code quality tools
- **ament_copyright** - License checking
- **ament_flake8** - Python code style
- **ament_pep257** - Python docstring checking

## Platform & Deployment

### Target Hardware
- **RK3588** - Primary deployment platform (ARM64/aarch64)
- **I2C4 Bus** - Hardware sensor communication interface
- **x86_64** - Development and simulation
- **Embedded Linux** - Real-time automotive systems
- **TriCore MCU** - Slave hardware communication via UART4

### Containerization
- **Docker** support for deployment
- **ROS2 launch** system for orchestration
- **systemd** integration for service management

### Security
- **Cryptography** library for data protection
- **TLS/SSL** for secure communications
- **Role-based access** control

## External Integrations

### Mapping & Navigation
- **OpenStreetMap (OSM)** - Geographic data
- **Mapbox** - Commercial mapping services
- **OpenDRIVE** - Road network descriptions

### Simulation
- **CARLA** - Autonomous driving simulator  
- **MetaDrive** - Driving scenario simulation
- **Autoware** - Open-source autonomous driving

### Cloud Services
- **AWS** integration via boto3
- **Remote telemetry** and monitoring
- **Over-the-air updates**

## Version Compatibility

- **Python**: 3.10+ required
- **ROS2**: Humble distribution (target platform)
- **Qt**: 5.15.x series
- **OpenCV**: 4.x series
- **NumPy**: 1.26.x for consistency

## ROS2 Sensing Architecture Implementation

### Threading Patterns
- **Simple Threading Locks** - Independent threading.Lock() for each I2C sensor
- **Producer-Consumer Pattern** - Hardware capture threads decoupled from ROS publishing
- **Adafruit/DFRobot Pattern** - Proven hardware integration patterns from submodules

### Message Architecture
- **Autoware Alignment** - `/sensing/` namespace for new implementations
- **Standard Messages** - sensor_msgs, geometry_msgs without vendor prefixes
- **Compatibility Bridges** - Maintain existing topics during migration
- **Consumer Safety** - Zero regression for perception system dependencies

### Hardware Integration Approach
- **Proven Patterns** - Use existing working implementations from submodules
- **MicroROS UART4** - Leverage existing 115200 bps TriCore communication
- **Independent I2C** - Each sensor maintains separate SMBus connection
- **Error Handling** - Exponential backoff retry patterns from RK3588 examples