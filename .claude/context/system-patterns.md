---
created: 2025-08-29T18:15:20Z
last_updated: 2025-08-29T18:15:20Z
version: 1.0
author: Claude Code PM System
---

# System Patterns and Architecture

## Core Architectural Patterns

### 1. Layered Architecture
The system follows a clear layered architecture with defined boundaries:
- **Presentation Layer** (PyQt5 GUI)
- **Application Layer** (Business logic and orchestration)
- **Domain Layer** (Core autonomous driving logic)
- **Infrastructure Layer** (Hardware interfaces, ROS2, I/O)

### 2. Publisher-Subscriber Pattern (ROS2)
- **Decoupled Communication** - Components communicate via ROS2 topics
- **Scalable Message Flow** - Easy to add/remove components
- **Real-time Processing** - Asynchronous message handling
- **Data Distribution Service** - Reliable, high-performance messaging

### 3. Microservice Architecture
Each ROS2 package represents a focused microservice:
- **Single Responsibility** - Each node has one clear purpose
- **Independent Deployment** - Packages can be updated independently
- **Technology Diversity** - Mix of Python and C++ components
- **Service Discovery** - ROS2 handles node discovery and communication

## Design Patterns Implementation

### 4. Observer Pattern
- **Topic Subscriptions** - Nodes observe data streams
- **Event-driven Processing** - React to sensor data and state changes
- **Diagnostic Monitoring** - System health observation
- **UI Updates** - GUI responds to system state changes

### 5. Strategy Pattern
- **Algorithm Selection** - Multiple planning algorithms (A*, behavior-based)
- **Camera Processing** - Different detection strategies per camera
- **Tracking Methods** - Configurable tracking algorithms (ByteTrack, OC_SORT)
- **Hardware Adaptation** - Different strategies for various hardware platforms

### 6. Factory Pattern
- **Node Creation** - ROS2 launch files act as factories
- **Camera Instantiation** - Dynamic camera configuration
- **Sensor Management** - Hardware-specific sensor creation
- **Model Loading** - AI model factory based on hardware capabilities

### 7. Pipeline Pattern
- **Perception Pipeline** - Sequential processing stages
- **Planning Pipeline** - Multi-stage decision making
- **Data Processing** - Sensor fusion pipelines
- **Image Processing** - Computer vision processing chains

## System Integration Patterns

### 8. Bridge Pattern
- **System Integration** - `shared/bridge/` for inter-system communication
- **Protocol Translation** - Different communication protocols
- **Hardware Abstraction** - Abstract hardware interfaces
- **Legacy Integration** - Bridge to existing automotive systems

### 9. Adapter Pattern
- **Sensor Interfaces** - Standardized sensor data formats
- **Camera Adapters** - Multiple camera types with common interface
- **Network Adapters** - Different network protocols (CAN, I2C, UART)
- **AI Model Adapters** - Multiple AI frameworks with unified interface

### 10. Facade Pattern
- **System APIs** - Simplified interfaces for complex subsystems
- **Hardware Facades** - Simple interfaces to complex hardware
- **Launch Systems** - Easy system startup and configuration
- **GUI Interfaces** - Simplified control interfaces for operators

## Data Flow Patterns

### 11. Data Pipeline Architecture
```
Sensors → Processing → Fusion → Decision → Control → Actuation
```
- **Streaming Data** - Continuous sensor data flow
- **Real-time Processing** - Low-latency requirements
- **Data Fusion** - Multiple sensor integration
- **Temporal Consistency** - Synchronized data processing

### 12. Event Sourcing
- **ROS2 Bag Recording** - Complete system state capture
- **Diagnostic Logging** - Event-based system monitoring
- **Replay Capabilities** - Deterministic system replay
- **Audit Trails** - Complete action history

### 13. CQRS (Command Query Responsibility Segregation)
- **Command Processing** - Control commands and actions
- **Query Processing** - Status and telemetry requests
- **Separate Interfaces** - Different paths for reads and writes
- **Performance Optimization** - Optimized for different access patterns

## Error Handling Patterns

### 14. Circuit Breaker Pattern
- **Hardware Fault Tolerance** - Prevent cascading failures
- **Network Resilience** - Handle communication failures
- **Sensor Degradation** - Graceful handling of sensor failures
- **Recovery Mechanisms** - Automatic fault recovery

### 15. Retry Pattern
- **Network Operations** - Retry failed communications
- **Sensor Reading** - Handle temporary sensor failures
- **Model Inference** - Retry AI model processing
- **Exponential Backoff** - Progressive retry delays

### 16. Bulkhead Pattern
- **Resource Isolation** - Separate pools for different components
- **Process Isolation** - ROS2 nodes in separate processes
- **Thread Isolation** - Separate threading contexts
- **Memory Isolation** - Prevent memory leaks from affecting system

## Performance Patterns

### 17. Caching Pattern
- **Model Caching** - Cache loaded AI models
- **Configuration Caching** - Cache frequently accessed config
- **Computation Caching** - Cache expensive calculations
- **Resource Caching** - Cache hardware resources

### 18. Connection Pooling
- **Database Connections** - Pool database connections
- **Network Connections** - Reuse network connections
- **Hardware Resources** - Pool camera and sensor access
- **Process Resources** - Manage system resource allocation

### 19. Lazy Loading
- **Model Loading** - Load AI models on demand
- **Configuration Loading** - Load configs when needed
- **Resource Initialization** - Initialize resources lazily
- **GUI Components** - Load UI components on demand

## Safety and Reliability Patterns

### 20. Watchdog Pattern
- **System Monitoring** - Monitor system health
- **Deadlock Detection** - Detect and recover from deadlocks
- **Performance Monitoring** - Track system performance
- **Automatic Recovery** - Restart failed components

### 21. Redundancy Pattern
- **Sensor Redundancy** - Multiple sensors for critical data
- **Processing Redundancy** - Backup processing paths
- **Communication Redundancy** - Multiple communication channels
- **Fail-safe Defaults** - Safe default behaviors

### 22. Validation Pattern
- **Input Validation** - Validate all sensor inputs
- **Configuration Validation** - Validate system configuration
- **State Validation** - Validate system state transitions
- **Output Validation** - Validate control outputs

## Configuration Patterns

### 23. Configuration Management
- **Hierarchical Configuration** - Layered config system
- **Environment-specific** - Different configs per environment
- **Runtime Configuration** - Dynamic configuration changes
- **Version Control** - Configuration versioning and rollback

### 24. Feature Flags
- **Experimental Features** - Enable/disable features dynamically
- **Hardware-specific Features** - Enable features based on hardware
- **A/B Testing** - Test different algorithms
- **Gradual Rollout** - Progressive feature deployment

## Hardware Integration Patterns

### 25. Proven I2C Threading Pattern (ROS2 Sensing)
**Decision**: Adopted simple threading.Lock() approach based on submodule analysis
- **Independent Threading** - Each I2C sensor maintains separate SMBus connection
- **Simple Lock Protection** - Use threading.Lock() for thread-safe I2C access
- **Producer-Consumer Pattern** - Hardware capture threads decoupled from ROS publishing
- **Adafruit/DFRobot Pattern** - Proven hardware patterns from existing implementations

```python
class I2CSensorBase:
    def __init__(self, bus_number=1, address=0x68):
        self.i2c_bus = smbus.SMBus(bus_number)
        self.i2c_lock = threading.Lock()  # Simple, proven protection
        
    def read_data(self):
        with self.i2c_lock:  # Thread-safe I2C access
            return self.i2c_bus.read_byte_data(self.address, register)
```

### 26. MicroROS UART Integration Pattern
**Decision**: Leverage existing UART4 @ 115200 bps implementation
- **Existing Implementation** - Use proven microROS UART4 bridge
- **TriCore Communication** - Maintain working slave hardware communication
- **XRCE-DDS Bridge** - Preserve functional UART → ROS2 topic bridge
- **No Reinvention** - Avoid recreating working communication patterns

### 27. Consumer-Safe Migration Pattern
**Decision**: Implement compatibility bridges for perception system safety
- **Topic Preservation** - Maintain existing `/road_camera/`, `/front_left_camera/` topics
- **Compatibility Bridges** - Gradual migration to `/sensing/` namespace
- **Zero Regression** - No functionality loss during transition
- **Rollback Capability** - Ability to revert changes if needed

### 28. Autoware Alignment Pattern
**Decision**: Align with autoware ecosystem standards while preserving functionality
- **Standard Namespaces** - Use `/sensing/` namespace for new implementations
- **Standard Messages** - Use sensor_msgs, geometry_msgs without vendor prefixes
- **Ecosystem Tools** - Compatible with autoware universe tooling
- **Progressive Migration** - Gradual alignment without disrupting existing systems