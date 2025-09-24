---
name: ros2-sensing-folder-code
description: Realistic refactoring using proven hardware patterns from submodules for autoware alignment
status: backlog
created: 2025-08-29T18:17:27Z
updated: 2025-08-29T19:17:38Z
last_sync: 2025-08-29T19:17:38Z
github: https://github.com/JinshiChen1989/nagasware/issues/3
---

# PRD: ros2-sensing-folder-code

## Executive Summary

This PRD defines the realistic refactoring and autoware alignment of the existing ROS2 sensing system codebase located in `src/sensing/` using proven hardware implementation patterns discovered in submodules. The project adopts battle-tested approaches: simple I2C threading locks (Adafruit/DFRobot pattern), existing microROS UART4 @ 115200 bps communication (TriCore implementation), and producer-consumer threading (v4l2_camera pattern).

The sensing system includes 12+ sensor packages that will be organized using realistic patterns: I2C sensors with independent threading.Lock() protection, existing UART4 microROS bridge for slave hardware communication, and autoware-aligned `/sensing/` topic namespaces with standard sensor_msgs. This initiative achieves autoware compatibility through proven, production-ready implementations rather than theoretical coordination systems.

## Problem Statement

### Current Challenges

1. **Inconsistent Threading Patterns**: I2C sensors lack thread-safe access patterns, requiring proven threading.Lock() implementation from submodule examples
2. **Consumer Dependency Risk**: Camera topics have hard-coded perception dependencies (`/road_camera/`, `/front_left_camera/`) that would break if changed during refactoring
3. **Autoware Misalignment**: Current topic namespaces and message patterns don't follow autoware standards (`/sensing/` namespace with standard sensor_msgs)
4. **Underutilized Existing Code**: MicroROS UART4 @ 115200 bps implementation exists but isn't properly integrated into autoware-aligned architecture
5. **Mixed Implementation Patterns**: Different sensor packages use varying approaches rather than following proven patterns from hardware examples
6. **Package Structure Inconsistency**: Sensor packages lack consistent organization compared to autoware ecosystem standards

### Why This Matters Now

- **Proven Pattern Adoption**: Submodules contain battle-tested hardware patterns (Adafruit/DFRobot) that can improve reliability
- **Autoware Ecosystem Integration**: Current architecture prevents integration with autoware universe tooling and standards
- **Consumer Safety**: Camera topic changes risk breaking perception system without compatibility bridges
- **Code Reuse Opportunity**: Existing microROS UART4 @ 115200 bps implementation can be leveraged for autoware alignment
- **Development Velocity**: Adopting proven patterns reduces development time and technical risk compared to custom solutions

## User Stories

### Primary Personas

**Robotics Engineer (Primary)**
- Needs to quickly understand sensor capabilities and integration points
- Wants consistent patterns for adding new sensors
- Requires reliable multi-sensor launch and configuration

**System Integrator (Secondary)**
- Needs clear dependency mapping between sensor packages
- Wants unified diagnostic and monitoring capabilities
- Requires standardized interfaces for system-level integration

**Maintenance Developer (Secondary)**  
- Needs consistent debugging and troubleshooting procedures
- Wants clear separation between hardware interfaces and application logic
- Requires standardized testing and validation procedures

### Detailed User Journeys

**UC1: Adding a New Sensor**
```
As a Robotics Engineer
When I need to integrate a new sensor (e.g., ultrasonic)
I want to follow a standardized template and process
So that integration is predictable and follows existing patterns
```

**Acceptance Criteria:**
- Clear sensor package template with standardized structure
- Documented integration process with shared components
- Automated tests validate new sensor follows conventions

**UC2: Multi-Sensor System Launch**
```
As a System Integrator  
When I need to start the complete sensing system
I want a single launch command with configurable sensor subsets
So that system startup is reliable and efficient
```

**Acceptance Criteria:**
- Single command launches all sensors with proper startup sequencing
- Configurable sensor groups (motion, environmental, navigation)
- Comprehensive diagnostic monitoring and health checks

**UC3: Sensor Debugging and Maintenance**
```
As a Maintenance Developer
When a sensor is malfunctioning or needs updates
I want consistent diagnostic tools and debugging procedures
So that I can quickly identify and resolve issues
```

**Acceptance Criteria:**
- Standardized diagnostic topics and message formats
- Consistent logging and error handling patterns
- Clear separation between hardware interface and business logic

## Requirements

### Functional Requirements

**FR1: Proven Pattern Implementation**
- I2C sensors using independent threading with simple locks (Adafruit/DFRobot pattern)
- Producer-consumer threading pattern for hardware capture (v4l2_camera pattern)
- Autoware-aligned folder structure and naming conventions following ecosystem standards

**FR2: Simple I2C Threading Protection**
- Each sensor maintains independent SMBus connection with threading.Lock() protection
- Error handling with exponential backoff retry (proven RK3588 hardware pattern)
- Background hardware capture threads decoupled from ROS publishing timing

**FR3: MicroROS UART Integration**
- Leverage existing UART4 @ 115200 bps TriCore communication implementation
- Integrate existing microROS bridge into autoware-aligned architecture
- Preserve functional UART → XRCE-DDS → ROS2 topic bridge patterns

**FR4: Consumer-Safe Migration Strategy**
- Preserve existing camera topic names during transition (`/road_camera/`, `/front_left_camera/`)
- Implement compatibility bridges for gradual migration to autoware namespaces
- Zero functionality regression for perception system integration

**FR5: Autoware-Aligned Message Architecture**
- Standard `/sensing/` namespace following autoware conventions
- Clean message types without vendor prefixes (sensor_msgs, geometry_msgs)
- Standard diagnostic patterns compatible with autoware ecosystem tools

### Non-Functional Requirements

**NFR1: Threading Pattern Reliability**
- I2C sensors use proven threading.Lock() patterns from submodule examples
- Individual sensor failures isolated through independent threading approach
- Hardware timing decoupled from ROS publishing through producer-consumer pattern

**NFR2: Consumer Compatibility Preservation**
- Zero perception system regression during camera topic migration
- Existing camera detection and tracking functionality maintained
- Gradual migration path with rollback capability through compatibility bridges

**NFR3: Autoware Ecosystem Integration**
- Topic namespaces and message formats compatible with autoware universe standards
- Standard sensor_msgs message types without vendor prefixes
- Launch system follows autoware sensor package patterns

**NFR4: Implementation Reusability**
- Leverage existing microROS UART4 @ 115200 bps implementation (no reinvention)
- Adopt proven hardware patterns from submodules (Adafruit/DFRobot/v4l2_camera)
- Maintain development velocity through battle-tested code patterns

## Success Criteria

### Measurable Outcomes

**Code Organization Metrics:**
- **Package Structure Consistency**: 100% of sensor packages follow identical structure
- **Code Duplication Reduction**: >80% reduction in duplicate I2C managers and utility code
- **Documentation Consistency**: 100% of packages have standardized README and API docs

**Development Velocity Metrics:**
- **New Sensor Integration Time**: <2 hours from template to working sensor package
- **Debugging Time**: 50% reduction in time to identify sensor issues through consistent diagnostics
- **Code Review Efficiency**: 40% reduction in review time through consistent patterns

**System Integration Metrics:**
- **Multi-Sensor Launch Success**: 99% success rate for complete system startup
- **Diagnostic Coverage**: 100% of sensors report standardized health status
- **Configuration Consistency**: Single configuration approach across all sensors

### Key Performance Indicators

1. **Development Experience Score**: Developer survey rating of ease-of-use (target: 4.5/5)
2. **Package Template Adoption**: 100% of new sensors use standardized template
3. **Shared Component Usage**: 100% of I2C sensors use shared I2C manager
4. **System Reliability**: <1 sensor-related integration failure per month
5. **Documentation Quality**: All packages pass documentation completeness checklist

## Constraints & Assumptions

### Technical Constraints

- Must maintain compatibility with existing ROS2 Humble distribution
- Cannot modify underlying hardware interfaces or sensor communication protocols
- Must preserve current sensor data rates and real-time performance characteristics
- Limited to existing I2C4 bus architecture for sensor communication

### Resource Constraints

- Reorganization must be completed without disrupting ongoing development
- Cannot require additional hardware or infrastructure changes
- Must work within current RK3588 platform capabilities and limitations
- Development time limited to prevent blocking other system components

### Assumptions

- All existing sensors are functioning correctly and should remain operational
- Current sensor message formats and topics are appropriate for system integration
- Existing launch files and configurations contain correct sensor parameters
- Hardware abstraction layer requirements will remain stable during reorganization

## Out of Scope

### Explicitly NOT Included

**Hardware Changes:**
- Modifications to sensor hardware or physical connections
- Changes to I2C bus configuration or addressing
- Addition of new sensors or hardware components

**Protocol Modifications:**
- Changes to sensor communication protocols (I2C, UART, CAN)
- Modifications to existing ROS2 message formats
- alterations to sensor data rates or timing requirements

**Feature Additions:**
- New sensor fusion algorithms or data processing capabilities
- Advanced calibration routines or sensor validation features  
- Integration with external services or cloud platforms

**System Architecture Changes:**
- Modifications to broader ROS2 system architecture
- Changes to perception or control system interfaces
- Alterations to launch system beyond sensing components

## Dependencies

### External Dependencies

**ROS2 Infrastructure:**
- ROS2 Humble distribution with standard sensor message packages
- sensor_msgs, geometry_msgs, and diagnostic_msgs packages
- Standard ROS2 launch system and parameter management

**Hardware Platform:**
- RK3588 development board with I2C4 bus access
- Existing sensor hardware (ICM-42688, VEML7700, SHT40-AD, AK09918, etc.)
- UART interfaces for GPS/RTK and CAN communication

**System Components:**
- Python 3.8+ with required packages (pyserial, numpy, threading)
- I2C bus drivers and permissions for sensor communication
- File system access for configuration and log management

### Internal Dependencies

**Codebase Dependencies:**
- Preservation of existing sensor driver functionality
- Maintenance of current topic publishing and subscription patterns
- Compatibility with existing launch files and configuration systems

**Team Dependencies:**
- Coordination with perception team for topic namespace changes
- Alignment with system integration team for launch system modifications
- Testing coordination with hardware team for sensor validation

**Documentation Dependencies:**
- Update of system-wide documentation to reflect new organization
- Training materials for new package structure and conventions
- API documentation updates for shared component usage

---

## Implementation Approach

### Phase 1: Proven Pattern Implementation (Week 1)
- Implement simple I2C threading locks following Adafruit/DFRobot patterns from submodules
- Validate existing microROS UART4 @ 115200 bps TriCore communication
- Create producer-consumer threading pattern based on v4l2_camera example
- Establish autoware-aligned message architecture with standard sensor_msgs

### Phase 2: I2C Sensor Migration (Week 2)
- Migrate IMU sensor using proven DFRobot ICM42688 threading pattern
- Migrate environmental sensors (compass, light, humidity) using independent threading
- Validate each sensor works independently with simple threading.Lock() protection
- Implement autoware `/sensing/` topic publishing with standard message types

### Phase 3: System Integration (Week 3)
- Align independent sensors (GPS, RTK, CAN) with autoware namespace conventions
- Implement camera compatibility bridges for safe perception migration
- Create unified launch system with modular sensor group selection
- Integrate existing microROS UART implementation into autoware-aligned architecture

### Phase 4: Consumer-Safe Migration & Validation (Week 4)
- Execute consumer-safe camera migration using compatibility bridges
- Comprehensive hardware validation with real RK3588 I2C/UART sensors
- Final integration testing with perception system (zero regression validation)
- System deployment readiness with proven hardware pattern validation

This realistic approach leverages battle-tested implementations from submodules while achieving autoware ecosystem compatibility through proven threading and communication patterns.