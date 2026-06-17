cmake_minimum_required(VERSION 3.10)
project(igris_c_masterarm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_link_options("-Wl,--no-as-needed")

find_package(OpenSSL REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

# Path to MasterArmSDK
set(MASTERARM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/MasterArmSDK
    CACHE PATH "Path to MasterArmSDK distribution"
)

# find_package(DynamixelSDK REQUIRED)

# Find dynamixel_sdk
set(DYNAMIXEL_SDK_DIR "" CACHE PATH "Path to dynamixel_sdk installation directory")
find_path(DYNAMIXEL_SDK_INCLUDE_DIR NAMES dynamixel_sdk/dynamixel_sdk.h
          PATHS ${DYNAMIXEL_SDK_DIR}/include /usr/local/include /usr/include
)
find_library(
  DYNAMIXEL_SDK_LIBRARY NAMES dynamixel_sdk dxl_sdk libdynamixel_sdk libdxl_x64_cpp.so
  PATHS ${DYNAMIXEL_SDK_DIR}/lib /usr/local/lib /usr/lib
)

message(STATUS "DYNAMIXEL_SDK_INCLUDE_DIR: ${DYNAMIXEL_SDK_INCLUDE_DIR}")
message(STATUS "DYNAMIXEL_SDK_LIBRARY: ${DYNAMIXEL_SDK_LIBRARY}")

if(NOT DYNAMIXEL_SDK_INCLUDE_DIR OR NOT DYNAMIXEL_SDK_LIBRARY)
  message(FATAL_ERROR "Could not find dynamixel_sdk. Please install it or set DYNAMIXEL_SDK_DIR.")
endif()

# MasterArmSDK paths
set(MASTERARM_SDK_LIBRARY ${MASTERARM_SDK_DIR}/lib/libMasterArmSDK.a)
set(MASTERARM_SDK_INCLUDE_DIR ${MASTERARM_SDK_DIR}/include)

# igris_c_sdk_public path
set(igris_c_sdk_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/igris_c_sdk_public/cyclonedds/lib/cmake/igris_c_sdk"
)
find_package(igris_c_sdk REQUIRED)

# Create imported target
add_library(
  MasterArmSDK STATIC IMPORTED
)
set_target_properties(
  MasterArmSDK
  PROPERTIES IMPORTED_LOCATION ${MASTERARM_SDK_LIBRARY}
             INTERFACE_INCLUDE_DIRECTORIES
             "${MASTERARM_SDK_INCLUDE_DIR};${DYNAMIXEL_SDK_INCLUDE_DIR}"
             INTERFACE_LINK_LIBRARIES ${DYNAMIXEL_SDK_LIBRARY}
)

add_executable(igris_c_masterarm_node igris_c_masterarm_node.cpp)
target_link_libraries(
  igris_c_masterarm_node MasterArmSDK igris_c_sdk::igris_c_sdk pthread rt
)

ament_target_dependencies(igris_c_masterarm_node rclcpp std_msgs)

install(TARGETS igris_c_masterarm_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
