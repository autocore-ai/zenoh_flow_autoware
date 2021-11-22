# User Guide

## Build from source

### Hardware and OS Requirement

* CPU: x86_64 4 Core or above
* RAM: 8G+
* Disk: 30G+ free space
* OS: Ubuntu 20.04

### Install ROS

* Add ROS 2 apt repositories

```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

* Install ROS2 packages

```bash
sudo apt update
sudo apt install ros-foxy-desktop
```

* Environment setup

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

* Install ROS2 build tools

```bash
sudo apt install build-essential clang-tidy cmake git python3-rosdep python3-vcstool python3-colcon-common-extensions
```

* Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Build Autoware.Auto

### Build Autoware for Zenoh-Flow

```bash
$ source /opt/AutowareAuto/setup.bash
$ cargo install cxxbridge-cmd --git "https://github.com/autocore-ai/cxx.git" --branch "autocore-dev"
$ colcon build --merge-install
```

## Run demo

## How it works
