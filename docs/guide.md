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

Autoware.Auto build and default install into `/opt/AutowareAuto`

```bash
$ git clone https://github.com/autocore-ai/AutowareAuto.git --depth 1 --branch zenoh-flow
$ sudo mkdir -p /opt/AutowareAuto
$ sudo chown -R $USER /opt/AutowareAuto
$ source /opt/ros/foxy/setup.bash
$ cd AutowareAuto
AutowareAuto$ git lfs install
AutowareAuto$ git lfs pull --include="*" --exclude=""
AutowareAuto$ vcs import < autoware.auto.foxy.repos
AutowareAuto$ rosdep install -y --from . --ignore-src --rosdistro foxy
AutowareAuto$ source /opt/ros/foxy/setup.bash
AutowareAuto$ colcon build --merge-install --install-base /opt/AutowareAuto
```

### Build Autoware for Zenoh-Flow

```bash
$ cargo install cxxbridge-cmd --git https://github.com/autocore-ai/cxx.git --branch autocore-dev
$ git clone https://github.com/autocore-ai/zenoh_flow_autoware.git --depth 1
$ cd zenoh_flow_autoware
zenoh_flow_autoware$ source /opt/AutowareAuto/setup.bash
zenoh_flow_autoware$ colcon build --merge-install
```

## Run demo
If zenoh-flow runtime is not installed, install and check it via below commands

```bash
$ cargo install runtime --git https://github.com/autocore-ai/zenoh-flow-examples.git --branch autocore-dev
$ runtime --help
dpn 0.1.0

USAGE:
    runtime [OPTIONS] --graph-file <graph-file> --runtime <runtime>

FLAGS:
    -h, --help       Prints help information
    -V, --version    Prints version information

OPTIONS:
    -g, --graph-file <graph-file>          
    -l, --loader_config <loader-config>    
    -o, --out-file <outfile>                [default: output.dot]
    -r, --runtime <runtime>                
```

In `zenoh_flow_autoware` folder, source the setup.bash and launch demo

```bash
zenoh_flow_autoware$ source /opt/AutowareAuto/setup.bash
zenoh_flow_autoware$ source install/setup.bash
zenoh_flow_autoware$ runtime --graph-file ./demo.yaml --runtime local
```

Launch a new terminal to open Rviz2

```bash
$ source /opt/AutowareAuto/setup.bash
$ ros2 launch autoware_auto_launch autoware_auto_visualization.launch.py
```

In Rviz2, `Ctrl + O` to open config `avp.rviz` in `zenoh_flow_autoware` folder.

After map loaded, drag start pose via `2D Pose Estimate` and end pose via `2D Goal Pose`, If all steps have no problem, tf from vehicle will start pilot with `zenoh_flow_autoware`.
