# MasterArmSDK

C++ SDK for controlling ROBROS Master Arm system with Dynamixel motors.

## Overview

**Author** : Myeongjin Lee menggu1234@robros.co.kr

**Version** : 1.0

## Directory Structure

```
dist/
├── lib/
│   └── libMasterArmSDK.a    # Static library
└── include/
    ├── rbrs_masterarm.hpp    # Main header
    └── utils.hpp             # Utility functions
```

## Dependencies

- **dynamixel_sdk**: Required for Dynamixel Protocol 2.0 support
  - Install from: https://github.com/ROBOTIS-GIT/DynamixelSDK
    ```
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK/c++/build/linux64
    make
    sudo make install
    sudo ldconfig
    ``` 
  - Install with ROS
    ```
    sudo apt install ros-{distro}-dynamixel-sdk
    ```
  - Or set `DYNAMIXEL_SDK_DIR` to the installation path

## Integration with CMake

### 1. Copy dist folder to your project

```bash
cp -r dist/ your_project/third_party/MasterArmSDK/
```

### 2. Add to your CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(YourProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to MasterArmSDK
set(MASTERARM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MasterArmSDK
    CACHE PATH "Path to MasterArmSDK distribution")

# Find dynamixel_sdk
set(DYNAMIXEL_SDK_DIR "" CACHE PATH "Path to dynamixel_sdk installation directory")
find_path(DYNAMIXEL_SDK_INCLUDE_DIR
  NAMES dynamixel_sdk/dynamixel_sdk.h
  PATHS ${DYNAMIXEL_SDK_DIR}/include /usr/local/include /usr/include
)
find_library(DYNAMIXEL_SDK_LIBRARY
  NAMES dynamixel_sdk dxl_sdk libdynamixel_sdk
  PATHS ${DYNAMIXEL_SDK_DIR}/lib /usr/local/lib /usr/lib
)

if(NOT DYNAMIXEL_SDK_INCLUDE_DIR OR NOT DYNAMIXEL_SDK_LIBRARY)
  message(FATAL_ERROR "Could not find dynamixel_sdk. Please install it or set DYNAMIXEL_SDK_DIR.")
endif()

# MasterArmSDK paths
set(MASTERARM_SDK_LIBRARY ${MASTERARM_SDK_DIR}/lib/libMasterArmSDK.a)
set(MASTERARM_SDK_INCLUDE_DIR ${MASTERARM_SDK_DIR}/include)

# Create imported target
add_library(MasterArmSDK STATIC IMPORTED)
set_target_properties(MasterArmSDK PROPERTIES
  IMPORTED_LOCATION ${MASTERARM_SDK_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES "${MASTERARM_SDK_INCLUDE_DIR};${DYNAMIXEL_SDK_INCLUDE_DIR}"
  INTERFACE_LINK_LIBRARIES ${DYNAMIXEL_SDK_LIBRARY}
)

# Your executable
add_executable(your_app your_source.cpp)
target_link_libraries(your_app MasterArmSDK)
```

## Basic Usage

```cpp
#include "rbrs_masterarm.hpp"
#include <iostream>
#include <vector>

int main() {
    // Initialize SDK
    // Parameters: port, baudrate, type (0=Protocol 2.0, 1=Protocol 1.0)
    RBRSMasterArm sdk("/dev/ttyUSB0", 1000000, 0);
    
    // Open connection
    sdk.open();
    
    // Set toggle current for hand motors (optional)
    sdk.set_toggle_current(50);
    
    // Read motor data
    sdk.read();
    
    // Get sensor data
    std::vector<F32> positions = sdk.get_positions();   // [rad]
    std::vector<F32> velocities = sdk.get_velocities(); // [rad/s]
    std::vector<I16> currents = sdk.get_currents();     // [mA]
    std::vector<U8> all_ids = sdk.get_all_ids();
    
    // Process data...
    
    // Close connection
    sdk.close();
    
    return 0;
}
```

## API Reference

### Constructor

```cpp
RBRSMasterArm(const std::string &port, int baudrate, int type = 0)
```

- `port`: Serial port path (e.g., "/dev/ttyUSB0")
- `baudrate`: Communication baud rate (e.g., 1000000)
- `type`: Protocol type. 0 = Dynamixel Protocol 2.0, 1 = Dynamixel Protocol 1.0 [Not in use]

### Methods

- `void open()` - Open serial connection and initialize motors
- `void close()` - Close connection and disable motors
- `void read()` - Read position, velocity, and current from all motors
- `void set_toggle_current(U16 current)` - Set target current for hand motors
- `std::vector<F32> get_positions() const` - Get positions in radians
- `std::vector<F32> get_velocities() const` - Get velocities in rad/s
- `std::vector<I16> get_currents() const` - Get currents in mA
- `std::vector<uint8_t> get_all_ids() const` - Get all motor IDs

### Motor IDs

- Right Arm: 11-17
- Left Arm: 21-27
- Right Hand: 18-19
- Left Hand: 28-29

## License

Copyright 2025 ROBROS INC.

