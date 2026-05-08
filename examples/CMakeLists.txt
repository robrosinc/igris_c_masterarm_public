cmake_minimum_required(VERSION 3.14)
project(igris_c_masterarm_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(UNIX AND NOT APPLE)
  add_link_options("-Wl,--no-as-needed")
endif()

set(igris_c_sdk_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/igris_c_sdk_public/lib/cmake/igris_c_sdk")
find_package(igris_c_sdk REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS SSL Crypto)
find_package(dynamixel_sdk REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 QUIET)

if(NOT TARGET glfw)
  message(STATUS "glfw3 not found. Downloading GLFW 3.4 from GitHub (first run only, ~10-30s)...")
  include(FetchContent)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )
  FetchContent_MakeAvailable(glfw)
  message(STATUS "GLFW ready.")
endif()

message(STATUS "Downloading ImGui v1.90.1 from GitHub (first run only, ~10-30s)...")
include(FetchContent)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90.1
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(imgui)
message(STATUS "ImGui ready.")

add_library(
  imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(
  imgui
  PUBLIC OpenGL::GL glfw
)

set(MASTERARM_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/MasterArmSDK" CACHE PATH "Path to MasterArmSDK distribution")

add_library(MasterArmSDK STATIC IMPORTED)
set_target_properties(
  MasterArmSDK
  PROPERTIES
    IMPORTED_LOCATION "${MASTERARM_SDK_DIR}/lib/libMasterArmSDK.a"
    INTERFACE_INCLUDE_DIRECTORIES "${MASTERARM_SDK_DIR}/include"
    INTERFACE_LINK_LIBRARIES dynamixel_sdk::dynamixel_sdk
)

add_executable(masterarm_gui_client masterarm_gui_client.cpp)
target_link_libraries(
  masterarm_gui_client
  igris_c_sdk::igris_c_sdk
  MasterArmSDK
  imgui
  OpenGL::GL
  glfw
  pthread
  rt
)

set_target_properties(
  masterarm_gui_client PROPERTIES INSTALL_RPATH "$ORIGIN/../lib" BUILD_RPATH_USE_ORIGIN ON
)

message(STATUS "Examples configured:")
message(STATUS "  - masterarm_gui_client: MasterArm-driven GUI client example")
