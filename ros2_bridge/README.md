# Robot MPC Ros2 Wrapper

Simple ros-humble wrapper for robotmpcs.


## Installation

Get the required submodules (forces_pro_server since ForcesPro currently does not support python 3.10).

```bash
git submodule update --init --recursive
git submodule update --recursive --remote
```

Navigate to root directory of `robot_mpcs`.
Every command assumes you are in the root directory of `robot_mpcs`.


Install `robot_mpcs` globally (or in the virtual environment if you use ros inside one).
```bash
pip3 install '.[agents]'
```

## Solver Generation

Run the following commands to automatically generate the solver. The solver is saved in the ros2_bridge/src/forces_pro_server/solver folder. Check if the solver was successfully generated.
In case the forces_pro_server is not required anymore the path has to be adapted.

```bash
cd ros2_bridge/robotmpcs_ros2/scripts
python3 make_solver.py ../config/jackal_mpc_config.yaml
```

## Forces Pro Server
Currently forcespro does not support Python 3.10. Thus, we run the solver inside a ROS2 service building on [forces_pro_server](https://github.com/oscardegroot/forces_pro_server).


To run the forces_pro_server, adapt the service node to find your solver. Change (ctrl+h) my_solver to <your_solver> in include/solver_service_node.h, src/solver_service_node.cpp, CMakelists.txt.
More information can be found [here](https://github.com/oscardegroot/forces_pro_server/blob/main/README.md)

## Visualization 
To visualize the robot's plan and other helpful things we make use of [ros_tools](https://github.com/oscardegroot/ros_tools/tree/ros2). Just add it to the workspace in /ros2_bridge/src and set use_visualization = True in [mpc_planner_node.py](src/robotmpcs_ros2/robotmpcs_ros2/mpc_planner_node.py).


## Build Workspace
Run the following command to build the workspace:
```bash
cd ros2_bridge
./build.sh
```
Don't forget to source your workspace in every terminal using
```bash
source install/setup.bash
```
or consider adding it to your `.bashrc` (`.zshrc`) file.
## Using the Planner in a Docker
In case the planner should be used within a docker see [here](doc/docker_doc.md).

## Running the MPC
Finally you made it this far. Now it is the time to run your MPC.
Assuming that you have a localization running which provides you with a StampedPose (check if topic is correct)

The following things need to be run:
- The forces_pro server
  ```
  ros2 launch forces_pro_server launch_server.launch.xml
  ```
- The main MPC node
    ```
  cd ros2_bridge/src/robotmpcs_ros2/launch
  ros2 launch ./jackal_mpc.launch.py 
  ```
- The action client used to provide a NavigateToGoal action using eviz
  ```
  cd ros2_bridge/src/robotmpcs_ros2/robotmpcs_ros
  python3 action_client.py 
  ```
- rviz
  ```
  rviz2
  ```


