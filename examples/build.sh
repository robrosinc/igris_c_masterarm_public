#!/bin/bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo -e "${YELLOW}=== Building IGRIS MasterArm Examples ===${NC}"

if [ -z "${AMENT_PREFIX_PATH}" ]; then
    if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        . "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        . "/opt/ros/jazzy/setup.bash"
    fi
fi

if [[ "$(uname -s)" == "Darwin" ]]; then
    if command -v brew >/dev/null 2>&1; then
        for pkg in glfw; do
            if ! brew list --versions "${pkg}" >/dev/null 2>&1; then
                echo -e "${YELLOW}Installing missing dependency: ${pkg}${NC}"
                brew install "${pkg}"
                if [ $? -ne 0 ]; then
                    echo -e "${RED}Failed to install ${pkg} with Homebrew.${NC}"
                    exit 1
                fi
            fi
        done
    else
        echo -e "${RED}Homebrew not found. Please install Homebrew first: https://brew.sh${NC}"
        exit 1
    fi
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

SDK_CMAKE_DIR="${SCRIPT_DIR}/../third_party/igris_c_sdk_public/lib/cmake/igris_c_sdk"

echo -e "${YELLOW}Configuring...${NC}"
if [ ! -f "${SDK_CMAKE_DIR}/igris_c_sdk-config.cmake" ]; then
    echo -e "${RED}igris_c_sdk_public not found at:${NC} ${SDK_CMAKE_DIR}"
    exit 1
fi

cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF -Digris_c_sdk_DIR="${SDK_CMAKE_DIR}"

if [ $? -ne 0 ]; then
    echo -e "${RED}CMake configuration failed!${NC}"
    exit 1
fi

echo -e "${YELLOW}Building...${NC}"
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif [[ "$(uname -s)" == "Darwin" ]]; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi

make -j"${JOBS}"

if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}=== Build completed ===${NC}"
echo -e "${GREEN}Executable:${NC}"
echo -e "  ${BUILD_DIR}/masterarm_gui_client"
echo ""
echo -e "${YELLOW}Usage:${NC}"
echo -e "  ./masterarm_gui_client <port> <baud> [domain_id] [namespace]"
echo -e "  ./masterarm_gui_client /dev/ttyUSB0 1000000 0"
