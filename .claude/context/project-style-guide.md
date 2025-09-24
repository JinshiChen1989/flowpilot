---
created: 2025-08-29T18:15:20Z
last_updated: 2025-08-29T18:15:20Z
version: 1.0
author: Claude Code PM System
---

# Project Style Guide

## Code Philosophy (from CLAUDE.md)

### Fundamental Principles
- **Think carefully and implement the most concise solution that changes as little code as possible**
- **NO PARTIAL IMPLEMENTATION** - Complete, production-ready code only
- **NO SIMPLIFICATION** - No placeholder comments or simplified versions
- **NO CODE DUPLICATION** - Reuse existing functions and constants
- **NO DEAD CODE** - Either use or delete completely
- **NO OVER-ENGINEERING** - Simple functions over complex abstractions

## Naming Conventions

### ROS2 Package Naming
- **snake_case** for all package names
- **Descriptive names** that clearly indicate purpose
- **Consistent suffixes** for package types:
  - `*_interfaces` - Message and service definitions
  - `*_helpers` - Utility and helper functions
  - `*_launch` - Launch file packages
  - `*_detection` - Object detection packages
  - `*_tracking` - Object tracking packages
  - `*_fusion` - Sensor fusion packages

### Node Naming
- **descriptive_node_name** pattern
- Clear indication of node's primary function
- Consistent with package naming

### File and Directory Structure
- **snake_case** for Python files and directories
- **CamelCase** for class names
- **UPPER_CASE** for constants and configuration keys
- **lowercase** for ROS2 topics and service names

## Python Coding Standards

### Code Style
- **PEP 8** compliance for Python code
- **Line length**: 88 characters (Black formatter standard)
- **Import organization**:
  1. Standard library imports
  2. Third-party imports
  3. ROS2 imports
  4. Local application imports

### Function and Class Design
- **Single Responsibility Principle** - One clear purpose per function/class
- **Comprehensive docstrings** - Google-style docstrings for all functions
- **Type hints** - Use type annotations for all function parameters and returns
- **Error handling** - Explicit error handling with appropriate exceptions

### Example Python Structure
```python
"""Module docstring describing purpose."""

import logging
from typing import Optional, List, Tuple

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from perception_interfaces.msg import DetectedObjects
from .utils import validation_helper


class PerceptionNode(Node):
    """Process camera data for object detection.
    
    This node subscribes to camera topics and publishes detected objects
    using AI-accelerated inference models.
    """
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.5) -> None:
        """Initialize perception node.
        
        Args:
            model_path: Path to the AI model file
            confidence_threshold: Minimum confidence for detections
        """
        super().__init__('perception_node')
        self._model_path = model_path
        self._threshold = confidence_threshold
```

## C++ Coding Standards

### Code Style
- **Google C++ Style Guide** as base reference
- **camelCase** for variable names
- **PascalCase** for class and function names
- **SCREAMING_SNAKE_CASE** for constants

### Memory Management
- **RAII principles** - Resource Acquisition Is Initialization
- **Smart pointers** preferred over raw pointers
- **Exception safety** - Strong exception safety guarantee
- **Resource cleanup** - Explicit cleanup in destructors

## ROS2 Specific Standards

### Message Design
- **Clear field names** with appropriate data types
- **Timestamp fields** for time-sensitive messages
- **Header fields** for coordinate frame information
- **Documentation** in .msg files explaining field purposes

### Node Design
- **Single responsibility** per node
- **Clean shutdown** handling
- **Parameter management** with validation
- **Quality of Service** appropriate for data type

### Launch File Standards
- **YAML configuration** for parameters
- **Modular design** allowing component reuse  
- **Environment-specific** configuration files
- **Clear documentation** of launch parameters

## Configuration Management

### Hierarchical Configuration
```
config/
├── global/           # System-wide settings
├── environments/     # Environment-specific settings
├── perception/       # Perception module settings
├── safety/          # Safety-critical parameters
└── vehicle/         # Vehicle-specific parameters
```

### Configuration File Format
- **YAML format** for all configuration files
- **Meaningful key names** with consistent naming
- **Default values** with inline documentation
- **Validation schemas** where appropriate

### Environment-Specific Configuration
- **Development** - Debug logging, relaxed thresholds
- **Testing** - Test-specific parameters and mock services
- **Production** - Optimized for performance and safety
- **Simulation** - Configured for simulator integration

## Testing Standards

### Test Organization
```
tests/
├── unit/            # Component unit tests
├── integration/     # System integration tests
├── hardware/        # Hardware-specific tests
├── fixtures/        # Test data and mock objects
└── suites/          # Test suite configurations
```

### Testing Requirements
- **IMPLEMENT TEST FOR EVERY FUNCTION** (from CLAUDE.md)
- **NO CHEATER TESTS** - Accurate, real-usage tests designed to reveal flaws
- **Verbose tests** for debugging purposes
- **Test data validation** - Realistic test scenarios
- **Performance benchmarks** - Ensure real-time requirements

### Test Naming
- **test_function_name_scenario()** pattern
- **Descriptive test names** explaining what is being tested
- **Given_When_Then** structure in test documentation

## Documentation Standards

### Code Documentation
- **Comprehensive docstrings** for all public functions and classes
- **Inline comments** for complex logic only
- **README files** for each package explaining purpose and usage
- **API documentation** generated from docstrings

### Architecture Documentation
- **System diagrams** showing component relationships  
- **Data flow diagrams** for understanding information flow
- **Sequence diagrams** for complex interactions
- **Decision records** documenting architectural choices

## Safety and Reliability Guidelines

### Input Validation
- **Validate all inputs** at system boundaries
- **Sanitize external data** before processing
- **Range checking** for numerical parameters
- **Type validation** for all function parameters

### Error Handling
- **Graceful degradation** under fault conditions
- **Comprehensive logging** for debugging and monitoring
- **Recovery mechanisms** for transient failures
- **Fail-safe defaults** for critical functions

### Resource Management
- **NO RESOURCE LEAKS** (from CLAUDE.md)
- **Close database connections** explicitly
- **Clear timeouts** and scheduled tasks
- **Remove event listeners** on cleanup
- **Clean up file handles** and network connections

## Performance Guidelines

### Real-Time Requirements
- **< 100ms latency** from perception to control
- **Deterministic behavior** in time-critical paths
- **Memory pool allocation** for real-time threads
- **Lock-free algorithms** where possible

### Optimization Strategies
- **Profile before optimizing** - Measure, don't assume
- **Vectorized operations** using NumPy and OpenCV
- **Hardware acceleration** - Utilize AI accelerators efficiently
- **Memory efficiency** - Minimize allocations in hot paths

## Git and Version Control

### Commit Standards
- **Conventional Commits** format for commit messages
- **Atomic commits** - One logical change per commit
- **Descriptive commit messages** explaining the "why"
- **Reference issues** in commit messages when applicable

### Branch Management
- **Feature branches** for development work
- **Descriptive branch names** indicating purpose
- **Clean history** through rebasing before merge
- **Automated testing** before merge to main

## Integration Guidelines

### Hardware Integration
- **Hardware abstraction layers** for different platforms
- **Configuration-driven** hardware selection
- **Graceful fallbacks** when hardware unavailable
- **Validation** of hardware capabilities at startup

### External Dependencies
- **Explicit dependency management** in requirements files
- **Version pinning** for production deployments
- **Fallback mechanisms** for optional dependencies
- **License compatibility** verification

This style guide reflects the project's commitment to **safety-critical, real-time autonomous vehicle software** with emphasis on **reliability, maintainability, and performance**.