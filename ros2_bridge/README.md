# Robot MPC Ros2 Wrapper

Simple ros-humble wrapper for boxer robot by clearpath.


## Installation

Navigate to root directory of `robot_mpcs`.
Every command assumes you are in the root directory of `robot_mpcs`.

Currently forcespro does not support Python 3.10. Thus, we run the solver inside a ROS2 service building on [forces_pro_server](https://github.com/oscardegroot/forces_pro_server).
Clone the forces_pro_server and boxer simulation environment using the provided submodules.

```bash
git submodule update --init --recursive
git submodule update --recursive --remote
```

To run the forces_pro_server, adapt the service node to find your solver. Change (ctrl+h) my_solver to <your_solver> in include/solver_service_node.h, src/solver_service_node.cpp, CMakelists.txt

Install `robot_mpcs` globally (or in the virtual environment if you use ros inside one).
```bash
pip install -e . agents
```

Build workspace
```bash
cd ros2_bridge
./build.sh
```

Generate the solver. This automatically saves the solver in the forces_pro_server folder. In case the forces_pro_server is not required anymore the path has to be adapted.
```bash
cd ros2_bridge
source source install/setup.bash
cd robotmpcs_ros2/scripts
python3 make_solver.py ../config/jackal_mpc_config.yaml
```

In case the planner should be used within a docker see [here](doc/docker_doc.md).

To launch the forces_pro_service node run in a sourced terminal


```
ros2 launch forces_pro_server launch_server.launch.xml
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

commandline
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
