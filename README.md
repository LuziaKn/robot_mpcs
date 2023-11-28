# Robot mpcs
This repository presents an implementation of Model Predictive Control (MPC) for a variety of robotic systems and tasks. The implementation is completely independent of the Robot Operating System (ROS) but can be combined with ROS1 via our [ROS1-bridge](ros_bridge/README.md) and with ROS2 via our [ROS2-bridge](ros2_bridge/README.md).
Currently, we use Embotechs ForcesPro solver to solve the MPC problem. More information can be found [here](https://forces.embotech.com/). 

## Install forces pro
You have to request a license for [forcespro](https://forces.embotech.com/) and install it
according to their [documentation](https://forces.embotech.com/Documentation/installation/obtaining.html#sec-obtaining).
Helful information about assigning the license to your computer can be found [here](https://my.embotech.com/manual/system_information).

To make use of the ForcesPro solver with python follow their instructions [here](https://forces.embotech.com/Documentation/installation/python.html#python).

The location of the python package `forcespro` must also be included in your python path.
```bash
export PYTHONPATH="${PYTHONPATH}:/path/to/forces/pro"
```
Consider adding it to your `.bashrc` (`.zshrc`) file

## Install using poetry
Then you can install this package using [poetry](https://python-poetry.org/docs/) within a
virtual environment.
```bash
poetry install -E agents
poetry shell
```
Now you are in the virtual environment with everything installed.

## Install globally using pip
```bash
pip3 install .
```
If you want to test the mpc solvers you need to install additional dependencies.
```bash
pip3 install '.[agents]'
```

## Examples
```
cd examples
python3 makeSolver.py <path/to/config/file>
python3 <robot/type>_example.py <path/to/config/file>
```

