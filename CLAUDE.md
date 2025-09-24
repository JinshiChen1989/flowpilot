# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FlowPilot is an autonomous driving software system (fork of OpenPilot) providing ADAS capabilities including lane centering, adaptive cruise control, and other self-driving features. It's ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY.

## Development Commands

### Build System
- `scons` - Build native C/C++ components using SCons build system
- `scons --test` - Build with test files
- `scons --asan` - Build with AddressSanitizer
- `scons --ubsan` - Build with UndefinedBehaviorSanitizer

### Python Environment
- `pipenv install` - Install Python dependencies (uses Pipfile)
- `pipenv shell` - Activate virtual environment
- `pip install -e .` - Install FlowPilot package in development mode

### Launch Scripts
- `./launch_flowpilot.sh` - Standard launch (tmux with flowinit)
- `./launch_flowpilot_full.sh` - Full launch with build step (`scons && flowinit`)

### Individual Daemons (via Pipfile scripts)
- `controlsd` - Main vehicle control daemon
- `plannerd` - Path planning and trajectory generation
- `calibrationd` - Camera and sensor calibration
- `flowinit` - Process manager and orchestration

### Code Quality
- `flake8` - Python linting (configured in Pipfile)
- `pylint` - Advanced Python analysis
- `pre-commit run --all-files` - Run pre-commit hooks
- Tests are run via `nose` framework

## Architecture Overview

### Core Message-Passing Architecture
- **ZeroMQ (pyzmq)**: Inter-process communication between daemons
- **Cap'n Proto**: High-performance serialization for message passing
- **Environment Variables**: Control communication protocol (`ZMQ_MESSAGING_PROTOCOL="TCP"`)

### Key System Components

#### Control & Planning (`selfdrive/controls/`)
- `controlsd.py`: Main control daemon handling steering/acceleration/braking
- `plannerd.py`: Path planning using Model Predictive Control (MPC)
- `lib/`: Control algorithms using CasADi and Acados optimization libraries

#### Perception & Localization
- `modeld/`: Machine learning models for computer vision
- `locationd/`: GPS, localization, sensor fusion with Extended Kalman Filter
- `calibration/`: Camera and sensor calibration systems

#### Hardware Interface
- `boardd/pandad.py`: Hardware abstraction via Panda device
- `car/`: Vehicle-specific CAN interfaces and configurations
- Environment variables control sensor modes (`NOSENSOR`, `USE_GPU`, `SIMULATION`)

#### System Services
- `manager/flowinitd.py`: Process management and health monitoring
- `thermald/`: Thermal management for hardware protection
- `loggerd/`: Data logging and telemetry collection

### Multi-Language Build System
- **Python**: Main application logic with Cython extensions
- **C/C++**: Performance-critical components (C11/C++17 standards)
- **Android/Java**: Mobile application via Gradle build system
- **SCons**: Primary build orchestration with complex dependency management

### Key Dependencies
- **NumPy/SciPy**: Numerical computing and scientific operations
- **CasADi/Acados**: Nonlinear optimization for Model Predictive Control
- **OpenCV**: Computer vision (implied from usage patterns)
- **LMDB**: Memory-mapped database for parameter storage
- **Rednose**: Extended Kalman Filter implementation

## Development Environment

### Launch Configuration
Environment variables in launch scripts control system behavior:
- `SIMULATION="0/1"`: Toggle simulation vs hardware mode
- `USE_GPU="0/1"`: Enable GPU acceleration
- `NOSENSOR="0/1"`: Disable sensor inputs for testing
- `FINGERPRINT`: Override vehicle detection
- `ZMQ_MESSAGING_PROTOCOL`: Communication protocol selection

### Testing
- Test files located in `panda/tests/`, `opendbc/can/tests/`
- Use `nose` test runner with `parameterized` for test cases
- Safety-critical testing for automotive compliance
- Individual daemon testing via console entry points

### Process Management
- **tmux**: Session management for multiple daemons
- **flowinit**: Central process orchestrator with health monitoring
- Daemons communicate via ZeroMQ message passing
- Clean shutdown and restart capabilities