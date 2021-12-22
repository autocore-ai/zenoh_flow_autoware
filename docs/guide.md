# User Guide

All those steps tested on [*`GitHub Actions`*](https://github.com/autocore-ai/zenoh_flow_autoware/actions/workflows/test.yml) test workflow

## Build from source

### Hardware and OS Requirement

* CPU: x86_64 4 Core or above
* RAM: 8G+
* Disk: 30G+ free space
* OS: Ubuntu 20.04

### Install ROS2

1. Add ROS 2 apt repositories

    ```bash
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

1. Install ROS2 packages

    ```bash
    sudo apt update
    sudo apt install ros-foxy-desktop
    ```

1. Environment setup

    ```bash
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

1. Install ROS2 build tools

    ```bash
    sudo apt install build-essential clang-tidy cmake git python3-rosdep python3-vcstool python3-colcon-common-extensions
    ```

1. Initialize rosdep

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
1. If zenoh-flow runtime is not installed, install and check it with below commands

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

1. In `zenoh_flow_autoware` folder, source the setup.bash and launch demo

    ```bash
    zenoh_flow_autoware$ source /opt/AutowareAuto/setup.bash
    zenoh_flow_autoware$ source install/setup.bash
    zenoh_flow_autoware$ runtime --graph-file ./demo.yaml --runtime local
    ```

1. When the graph file launched, the output likes:

    ```bash
    InitPoseReceiver
    GoalPoseReceiver
    NDTMapPublisherNode
    Lanelet2MapProviderNode
    [INFO] [1640084984.811272592] [Lanelet2MapProvider]: Waiting for earth to map transform - please start ndt_map_publisher .... : "earth" passed to lookupTransform argument target_frame does not exist. 
    Lanelet2MapVisualizer
    LanePlannerNode
    ParkingPlannerNode
    Lanelet2GlobalPlannerNode
    BehaviorPlannerNode
    PurePursuitNode
    SimplePlanningSimulator
    [INFO] [1640084985.554512447] [simple_planning_simulator]: vehicle_model_type = IDEAL_STEER_VEL
    [INFO] [1640084985.555540789] [simple_planning_simulator]: initialize_source : INITIAL_POSE_TOPIC
    Simulator waiting initialization...
    Simulator waiting initialization...
    Simulator waiting initialization...
    Simulator waiting initialization...
    ...
    ```

1. Launch a new terminal to open Rviz2

    ```bash
    $ source /opt/AutowareAuto/setup.bash
    $ ros2 launch autoware_auto_launch autoware_auto_visualization.launch.py
    ```

1. In Rviz2, *`Ctrl + O`* to open config *`avp.rviz`* in `zenoh_flow_autoware` folder.

1. After map loaded, drag start pose with *`2D Pose Estimate`* and end pose with *`2D Goal Pose`*, If all steps have no problem the vehicle will start pilot with `zenoh_flow_autoware`.
