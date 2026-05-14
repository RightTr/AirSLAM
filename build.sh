#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="HUMBLE"

pushd "$(pwd)" > /dev/null
cd "$(dirname "$0")"
echo "Working Path: $(pwd)"

ROS_VERSION=""
ROS_DISTRO_TAG=""

if [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
elif [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_DISTRO_TAG=${VERSION_HUMBLE}
else
    echo "Usage: ./build.sh ROS1|ROS2|humble"
    popd > /dev/null
    exit 1
fi

echo "ROS version is: ${ROS_VERSION}"
if [ -n "${ROS_DISTRO_TAG}" ]; then
    echo "ROS distro tag: ${ROS_DISTRO_TAG}"
fi

rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

if [ "${ROS_VERSION}" = "${VERSION_ROS1}" ]; then
    cp -f package_ROS1.xml package.xml
else
    cp -f package_ROS2.xml package.xml
fi

pushd "$(pwd)" > /dev/null
cd ../../
if [ "${ROS_VERSION}" = "${VERSION_ROS1}" ]; then
    catkin_make -DROS_EDITION=${VERSION_ROS1}
else
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2}
fi
popd > /dev/null

popd > /dev/null
