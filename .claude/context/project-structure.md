---
created: 2025-08-29T18:15:20Z
last_updated: 2025-08-29T18:15:20Z
version: 1.0
author: Claude Code PM System
---

# Project Structure

## Root Directory Organization

```
nagasware/
├── bootloader/          # System bootloader and initialization
├── config/              # Global configuration files
├── docs/                # Project documentation
├── logs/                # Runtime log storage
├── models/              # AI/ML model files (HEF, ONNX, RKNN formats)
├── pyqt5/               # PyQt5 GUI application framework
├── shared/              # Shared resources and runtime components
├── src/                 # Main source code (ROS2 packages)
├── submodules/          # Git submodules for external dependencies
├── tests/               # Test suite organization
└── .vscode/             # Development environment settings
```

## Core Architecture Layers

### 1. System Layer (`bootloader/`, `config/`)
- **bootloader/** - System initialization, GUI framework setup
- **config/** - Environment and system-wide configuration

### 2. Application Layer (`pyqt5/`)
- **domains/** - Business logic domains (graphics, hardware, navigation, vehicle)
- **src/core/** - Core application framework
- **src/ui/** - User interface components  
- **src/views/** - Application view controllers
- **tests/** - Comprehensive test coverage (unit, integration, e2e)

### 3. Runtime Layer (`src/`)
- **common/** - Shared interfaces, messages, utilities
- **sensing/** - Hardware sensor integration (cameras, GPS, IMU, CAN)
- **perception/** - Computer vision and AI processing
- **planning/** - Path planning and decision making
- **control/** - Vehicle control systems
- **safety/** - Safety monitoring and emergency systems
- **system/** - System monitoring and management

### 4. Integration Layer (`shared/`, `submodules/`)
- **shared/bridge/** - Inter-system communication
- **shared/runtime/** - Runtime state management
- **submodules/ai/** - AI/ML frameworks and models
- **submodules/hardware/** - Hardware abstraction libraries

## Key Subsystem Details

### Perception Pipeline (`src/perception/`)
- Multi-camera detection and tracking (6 cameras: front-left/right, rear-left/right, driver, road)
- Ground segmentation and occupancy mapping
- Stereo vision processing
- AI-powered object detection and fusion

### Planning System (`src/planning/`)
- A* path planning algorithm
- Behavior-based path planning
- Dynamic cruise and lateral control
- Navigation integration (OSM, Mapbox)

### Sensing Hardware (`src/sensing/`)
- Camera management and simulation
- CAN bus communication
- GPS/RTK positioning
- IMU sensor fusion
- Environmental sensors (humidity, light)

### Safety Systems (`src/safety/`)
- Emergency control mechanisms
- Safety monitoring and validation
- Safety-critical message handling

## Development Infrastructure

### Models (`models/`)
- **hef/** - Hailo AI accelerator models
- **onnx/** - ONNX format models for cross-platform inference
- **rknn/** - Rockchip NPU optimized models
- **s19/**, **tc275/** - Embedded system firmware

### Testing (`tests/`)
- **unit/** - Component unit tests
- **integration/** - System integration tests  
- **hardware/** - Hardware-specific tests
- **fixtures/** - Test data and mock objects

### Documentation (`docs/`)
- **architecture/** - System architecture documentation
- **implementation/** - Implementation guides
- **tracking/** - Project tracking and progress

### Project Management (`.claude/`)
- **prds/** - Product Requirements Documents
  - `ros2-sensing-folder-code.md` - ROS2 sensing system refactoring PRD
- **epics/** - Epic decomposition and task planning  
  - `ros2-sensing-folder-code/` - 9 realistic tasks for sensing system organization
- **context/** - Project context and documentation

## Naming Patterns

- **Packages**: snake_case ROS2 package naming
- **Nodes**: descriptive_node_name pattern
- **Interfaces**: *_interfaces packages for message definitions
- **Helpers**: *_helpers packages for utility functions
- **Launch**: *_launch packages for system startup scripts

## Module Dependencies

The system follows a layered dependency model:
1. **Hardware Layer** - Direct hardware interfaces
2. **Sensing Layer** - Sensor data processing  
3. **Perception Layer** - Environmental understanding
4. **Planning Layer** - Decision making
5. **Control Layer** - Vehicle actuation
6. **Safety Layer** - Cross-cutting safety concerns
7. **UI Layer** - Human-machine interface