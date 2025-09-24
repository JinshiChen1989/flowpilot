---
name: ros2-sensing-folder-code
status: backlog
created: 2025-08-29T18:21:56Z
updated: 2025-08-29T19:17:38Z
last_sync: 2025-08-29T19:17:38Z
progress: 0%
prd: .claude/prds/ros2-sensing-folder-code.md
github: https://github.com/JinshiChen1989/nagasware/issues/2
---

# Epic: ros2-sensing-folder-code

## Overview

Realistic refactoring and autoware alignment of the existing ROS2 sensing system based on proven hardware implementation patterns from submodules. This epic focuses on adopting battle-tested approaches: simple I2C threading locks (not complex coordination), existing microROS UART4 115200 bps patterns for slave hardware, and producer-consumer threading similar to v4l2_camera.

The implementation leverages existing working code: microROS UART4 @ 115200 bps for TriCore slave communication, independent I2C sensor threading (following Adafruit/DFRobot patterns), and autoware-aligned topic namespaces while preserving perception dependencies.

## Architecture Decisions

**Proven Hardware Pattern Adoption: Real-World Examples**
- I2C sensors use independent threading with simple locks (Adafruit/DFRobot pattern from submodules)
- MicroROS UART4 @ 115200 bps leverages existing TriCore communication (already implemented)
- Producer-consumer threading for sensors (v4l2_camera pattern from submodules)
- Autoware-aligned `/sensing/` namespace with standard sensor_msgs (no vendor prefixes)

**Simple I2C Resource Management: Threading Locks Pattern** 
- Each sensor maintains independent SMBus connection (proven in Adafruit examples)
- Thread-safe access using `threading.Lock()` per I2C bus (not complex coordination)
- Error handling with exponential backoff retry (pattern from RK3588 examples)
- Separate hardware capture threads decoupled from ROS publishing timing

**MicroROS UART Integration: Existing Implementation**
- UART4 @ 115200 bps for TriCore slave hardware (already working in codebase)
- Native XRCE protocol handling using existing `tricore_microros_node.py`
- Message converter patterns already established in `message_converter.py`
- Bridge from UART → XRCE-DDS → ROS2 topics (production-ready)

**Autoware Ecosystem Alignment: Standard Compliance**
- Topic namespaces follow autoware conventions (`/sensing/{type}/{name}/`)
- Standard ROS2 message types (sensor_msgs, geometry_msgs) without prefixes
- Consumer compatibility bridges for perception system safety

## Technical Approach

### Realistic I2C Management (Proven Patterns)

**Independent I2C Threading Pattern (From Adafruit/DFRobot Examples)**
```python
class SensorNode(rclpy.Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Independent I2C connection per sensor (proven pattern)
        self.i2c_bus = smbus.SMBus(1)  # I2C1 bus
        self.i2c_lock = threading.Lock()  # Simple threading protection
        
        # Producer thread for hardware I/O (v4l2_camera pattern)
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.data_queue = queue.Queue(maxsize=10)
        
        # ROS2 timer for publishing (decoupled from hardware timing)
        self.create_timer(0.02, self._publish_callback)  # 50Hz ROS rate
        
    def _capture_loop(self):
        """Background thread for hardware capture (like v4l2_camera)"""
        while rclpy.ok():
            try:
                with self.i2c_lock:  # Simple lock protection
                    data = self.i2c_bus.read_byte_data(0x68, 0x3B)
                self.data_queue.put(data, timeout=0.001)
                time.sleep(0.01)  # Hardware sampling rate (100Hz)
            except Exception as e:
                self.get_logger().warn(f"I2C read failed: {e}")
                time.sleep(0.1)  # Exponential backoff on error
```

**MicroROS UART Integration (Existing Implementation)**
- UART4 @ 115200 bps communication with TriCore slave hardware
- Existing `TriCoreMicroROSNode` bridges UART → ROS2 topics  
- Native XRCE protocol handling already production-ready
- Message conversion patterns established and tested

### Autoware-Aligned Message Architecture

**Clean Message Standards (No Vendor Prefixes)**
```yaml
# I2C Sensors (coordinated via shared manager)
/sensing/imu/data           → sensor_msgs/Imu
/sensing/compass/magnetic   → sensor_msgs/MagneticField  
/sensing/light/illuminance  → sensor_msgs/Illuminance
/sensing/humidity/data      → sensor_msgs/RelativeHumidity

# Independent Sensors
/sensing/camera/road_camera/image_raw    → sensor_msgs/Image
/sensing/camera/road_camera/camera_info  → sensor_msgs/CameraInfo
/sensing/gps/fix                         → sensor_msgs/NavSatFix
/sensing/can/vehicle_status              → Custom vehicle messages

# Compatibility Bridges (during migration)
/road_camera/image_raw → /sensing/camera/road_camera/image_raw
```

**Autoware Diagnostic Integration**
- `/diagnostics/i2c_bus` for shared resource health monitoring
- `/diagnostics/sensors/{sensor_name}` following autoware conventions
- Bus utilization metrics and timing analysis

### Resource-Aware Launch System

**Hardware-Constrained Launch Architecture**
```
sensing_system.launch.py
├── shared_i2c_sensors.launch.py    # IMU, compass, light, humidity (coordinated)
├── independent_sensors.launch.py   # GPS, RTK, CAN (parallel startup)
├── camera_system.launch.py         # Cameras (perception-critical)  
└── autoware_bridge.launch.py       # Compatibility layer
```

**Coordinated I2C Sensor Startup**
- I2C manager starts first, then all I2C sensors simultaneously
- Priority-based bus access prevents startup conflicts
- Graceful degradation if individual sensors fail during initialization

## Implementation Strategy

**Phase 1: I2C Infrastructure Foundation (Controlled Risk)**
- Implement thread-safe I2C manager with priority-based scheduling
- Validate bus coordination without modifying existing sensor packages
- Create I2C sensor startup coordination and error recovery mechanisms

**Phase 2: Shared I2C Sensor Migration (Medium Risk)**  
- Migrate IMU, compass, light, humidity to use shared I2C manager
- Validate 50Hz IMU performance with concurrent sensor operation
- Test bus contention scenarios and automatic recovery procedures

**Phase 3: Independent Sensor Standardization (Low Risk)**
- Refactor GPS, RTK, CAN packages using autoware-aligned patterns
- Implement standard `/sensing/` namespace and clean message types
- These sensors have no shared resources or consumer dependencies

**Phase 4: Consumer-Safe Camera Migration (High Risk)**
- Analyze all perception system dependencies on camera topics
- Implement compatibility bridges preserving existing topic names
- Gradual migration with extensive testing and rollback procedures

**Risk Mitigation Strategies:**
- I2C bus backup/recovery procedures for shared resource failures
- Perception regression testing suite for camera topic changes
- Feature flags enabling rollback to original sensor implementations
- Hardware-in-loop testing for I2C timing and bus contention validation

## Task Breakdown Preview

High-level task categories that will be created:

- [ ] **I2C Threading Pattern Implementation**: Simple threading locks for I2C sensors (Adafruit/DFRobot pattern)
- [ ] **Producer-Consumer Thread Migration**: Background hardware capture + ROS publishing (v4l2_camera pattern)
- [ ] **MicroROS UART Integration**: Leverage existing UART4 115200 bps TriCore communication
- [ ] **Independent Sensor Autoware Alignment**: Standardize GPS, RTK, CAN with `/sensing/` namespace
- [ ] **Consumer-Safe Camera Migration**: Preserve perception dependencies with compatibility bridges
- [ ] **Autoware Message Architecture**: Standard sensor_msgs without vendor prefixes  
- [ ] **Realistic Launch System**: Independent sensor launch following proven patterns
- [ ] **Hardware Integration Testing**: Validate patterns work with real RK3588 I2C/UART hardware

## Dependencies

**Hardware Dependencies:**
- I2C4 bus access and timing characteristics (critical for shared sensor coordination)
- Existing sensor hardware: ICM-42688 IMU, AK09918 compass, VEML7700 light, SHT40-AD humidity
- RK3588 platform capabilities and I2C driver stability

**Consumer System Dependencies:**
- Perception system topic expectations (`/road_camera/`, `/front_left_camera/`) must be preserved
- Camera detection and tracking system functionality cannot regress during migration
- Autoware bridge compatibility with new message flows

**Critical Path Dependencies:**
- I2C manager implementation and validation before any sensor migration
- Consumer dependency analysis before camera topic namespace changes
- Hardware-in-loop testing for I2C timing before production deployment

## Success Criteria (Technical)

**Hardware Resource Efficiency:**
- I2C bus contention reduced by >60% through priority-based coordination
- IMU maintains 50Hz data rate with zero timing violations
- I2C4 bus recovery time <500ms after sensor failure isolation

**Consumer Compatibility:**
- Zero perception system regression during camera migration
- All existing detection and tracking functionality preserved
- Compatibility bridge latency <1ms for topic translations

**Autoware Ecosystem Integration:**
- 100% topic namespace alignment with autoware universe conventions
- Clean message architecture without vendor-specific prefixes
- Diagnostic patterns compatible with autoware health monitoring tools

## Estimated Effort

**Overall Timeline: 4 weeks (realistic based on proven patterns)**

**Phase 1: Threading Pattern Implementation (1 week)**
- Implement simple I2C threading locks (Adafruit/DFRobot pattern): 0.5 week
- Producer-consumer thread pattern for sensors (v4l2_camera pattern): 0.5 week

**Phase 2: Sensor Migration (1.5 weeks)**
- I2C sensor threading migration (IMU, compass, light, humidity): 1 week
- Independent sensor autoware alignment (GPS, RTK, CAN): 0.5 week

**Phase 3: MicroROS Integration (0.5 week)**
- Leverage existing UART4 115200 bps TriCore communication: 0.5 week

**Phase 4: Consumer-Safe Camera Migration (1 week)**
- Camera topic compatibility bridges and perception regression testing: 1 week

**Resource Requirements:**
- 1 developer familiar with Python rclpy and basic I2C/threading patterns
- Hardware access for validation with real RK3588 sensors
- Coordination with perception team for camera topic migration

**Critical Path Items:**
1. Simple threading pattern adoption (low risk - proven patterns)
2. Consumer-safe camera migration (perception system dependency)  
3. Hardware validation with existing microROS UART4 communication

This realistic approach leverages proven hardware patterns from submodules while achieving autoware ecosystem compatibility with minimal complexity.

## Tasks Created (Revised - Based on Proven Hardware Patterns)

- [ ] 001.md - I2C Threading Pattern Implementation (parallel: true)
- [ ] 002.md - MicroROS UART4 Integration Analysis (parallel: true)
- [ ] 003.md - IMU Sensor Threading Migration (parallel: false)
- [ ] 004.md - Environmental Sensors Threading Migration (parallel: false)
- [ ] 005.md - Independent Sensors Autoware Alignment (parallel: true)
- [ ] 006.md - Camera Compatibility Bridge Implementation (parallel: true)
- [ ] 007.md - Consumer-Safe Camera Migration (parallel: false)
- [ ] 008.md - Unified Sensing Launch System (parallel: false)
- [ ] 009.md - Hardware Integration Validation (parallel: false)

**Total tasks: 9** (reduced from 11)
**Parallel tasks: 4** (can run simultaneously)
**Sequential tasks: 5** (dependency chain)
**Estimated realistic effort: 172 hours (~4.3 weeks)**

**Key Reductions:**
- Simple I2C threading locks vs complex coordination (50% effort reduction)
- Reuse existing microROS UART4 @ 115200 bps (no reimplementation needed)
- Producer-consumer pattern from proven v4l2_camera example
- Standard sensor_msgs without vendor prefixes (autoware-aligned)