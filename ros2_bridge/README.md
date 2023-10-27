# Robot MPC Ros Wrapper

Simple ros-noetic wrapper for boxer robot by clearpath.


## Installation

Navigate to root directory of `robot_mpcs`.
Every command assumes you are in the root directory of `robot_mpcs`.

Clone the boxer simulation environment using the provided submodules.

```bash
git submodule update --init --recursive
git submodule update --recursive --remote
```

Install `robot_mpcs` globally (or in the virtual environment if you use ros inside one).
```bash
pip install -e .
```

Build catkin workspace
```bash
cd ros_bridge
colcon build
```

Generate the solver.
```bash
cd ros_bridge
source source install/local_setup.bash
cd robotmpcs_ros/script
python3 make_solver.py ../config/jackal_mpc_config.yaml
```
```commandline
sudo apt install ros-galactic-velodyne-description
sudo apt install ros-galactic-robot-localization
sudo apt install ros-galactic-imu-filter-madgwick
sudo apt install ros-galactic-controller-manager
interactive_marker_twist_server
twist_mux
source /usr/share/gazebo/setup.sh
sudo apt install ros-<distro>-ros2-control
sudo apt install ros-galactic-gazebo-ros2-control
joint_state_broadcaster
sudo apt install ros-galactic-diff-drive-controller

```
Launch mpc planner
```bash
cd ros_bridge
source devel/setup.{zsh|bash}
roslaunch jackal_gazebo jackal_world.launch.py
roslaunch robotmpcs_ros jackal_mpc.launch
```

Now, you can see a topic to publish the goal to.

```bash
ros2 topic pub /mpc/goal std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.0, 1.0]"

```


