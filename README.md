# P-PUMA Control System

A ROS 2 C++ package implementing advanced path tracking control algorithms for autonomous vehicle guidance. The system provides multiple pursuit-based strategies (Pure Pursuit, LP Pursuit, ACC Pursuit) with configurable vehicle dynamics models.

## Overview

The **P-PUMA** control system is designed to guide autonomous vehicles along predefined paths using interchangable path-tracking algorithms. It integrates with ROS 2 for distributed computing and real-time control, offering flexible algorithm selection and tunable parameters for different vehicle configurations and driving conditions.

## Features

- **Multiple Path Tracking Algorithms**
  - **Pure Pursuit**: Classic pursuit algorithm for smooth path tracking
  - **LP Pursuit**: Low-Pass filtered pursuit for reduced steering noise
  - **ACC Pursuit**: Acceleration-based pursuit strategy + Low-Pass filtering

- **Vehicle Dynamics Models**
  - Dry surface dynamics
  - Wet surface dynamics
  - Configurable via VehicleConfig Class

- **Robust Control**
  - Mission-based speed limits
  - State validation and timeout detection
  - Emergency handling

## Project Structure

```
p-puma/
├── control_p2/                  # Main ROS 2 package
│   ├── include/control_p2/
│   │   ├── control_manager.hpp      # Core control logic management
│   │   ├── control_node.hpp         # ROS 2 node implementation
│   │   ├── options.hpp              # Configuration parameters
│   │   ├── utils.hpp                # Utility functions
│   │   ├── math/
│   │   │   ├── pure_pursuit.hpp
│   │   │   ├── lp_pursuit.hpp
│   │   │   └── acc_pursuit.hpp
│   │   └── model/
│   │       ├── vehicle_config.hpp
│   │       └── dry_model.hpp
│   ├── src/
│   │   ├── control_manager.cpp
│   │   ├── control_node.cpp
│   │   └── math/
│   │       ├── pure_pursuit.cpp
│   │       ├── lp_pursuit.cpp
│   │       └── acc_pursuit.cpp
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── LICENSE
├── test/                        # Unit tests
│   ├── test_manager.cpp
│   └── test_plan.cpp
└── README.md
```

## Dependencies

### Core Dependencies
- **ROS 2** (Humble or newer recommended)
- **ament_cmake**: Build system
- **rclcpp**: ROS 2 C++ client library
- **Eigen**: Linear algebra library
- **lart_msgs**: Custom message definitions (LART project)
- **tf2**: Coordinate transformation library

## Installation

### Prerequisites
Ensure you have ROS 2 installed and sourced:
```bash
source /opt/ros/<ros-distro>/setup.bash
```

### Build from Source
```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/yourusername/p-puma.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select p-puma

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Control Node
```bash
ros2 run p-puma control_p2
```

### ROS 2 Topics

#### Subscriptions (Inputs)
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/lart/state` | `lart_msgs/State` | Current vehicle state |
| `/lart/mission` | `lart_msgs/Mission` | Mission configuration |
| `/lart/path` | `lart_msgs/PathSpline` | Path to follow |
| `/lart/dynamics` | `lart_msgs/Dynamics` | Vehicle dynamics parameters |
| `/pose` | `geometry_msgs/PoseStamped` | Current vehicle pose |

#### Publications (Outputs)
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/lart/dynamics_cmd` | `lart_msgs/DynamicsCMD` | Control commands (speed, steering) |
| `/visualization_marker` | `visualization_msgs/Marker` | Visualization of target point |

## Configuration

### Algorithm Selection
Edit `control_p2/include/control_p2/options.hpp`:
```cpp
#define ALGORITHM  "math/lp_pursuit.hpp"
#define MODEL      "model/dry_model.hpp"
```

Available algorithms:
- `math/pure_pursuit.hpp`
- `math/lp_pursuit.hpp`
- `math/acc_pursuit.hpp`

### Speed Limits
Configure mission-based speed limits in `options.hpp`:
```cpp
#define DEFAULT_MAX_SPEED 2.0f  // m/s
#define ACC_SPEED 2.0f          // m/s
#define EBS_SPEED 2.0f          // m/s
```

### Control Frequency
```cpp
#define FREQUENCY 50  // Hz
```

## Algorithm Details

### Pure Pursuit
- Classic path-tracking algorithm

### LP Pursuit (Low-Pass)
- Enhanced Pure Pursuit with filtered steering
- Reduces high-frequency steering noise
- Smoother vehicle response

### ACC Pursuit (Acceleration Control)
- Combines low-pass path tracking with speed regulation
- PID-based speed controller

## Testing

Run unit tests:
```bash
colcon test --packages-select p-puma
```

## Acknowledgments

- Built for LART (Leiria Academic Racing Team)

---

**Last Updated**: March 2024  
**ROS 2 Version**: Humble (or newer)  
**Build Status**: [![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
