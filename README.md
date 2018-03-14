# An OpenAI gym extension for using Gazebo known as `gym-gazebo`

<!--[![alt tag](https://travis-ci.org/erlerobot/gym.svg?branch=master)](https://travis-ci.org/erlerobot/gym)-->

This work presents an extension of the initial OpenAI gym for robotics using ROS and Gazebo. A whitepaper about this work is available at https://arxiv.org/abs/1608.05742. Please use the following BibTex entry to cite our work:

```
@article{zamora2016extending,
  title={Extending the OpenAI Gym for robotics: a toolkit for reinforcement learning using ROS and Gazebo},
  author={Zamora, Iker and Lopez, Nestor Gonzalez and Vilches, Victor Mayoral and Cordero, Alejandro Hernandez},
  journal={arXiv preprint arXiv:1608.05742},
  year={2016}
}
```

-----


## Getting the Husky Environment Ready
### Requirement

**ROS Indigo** 
**Gazebo 7**


### Build and install gym-gazebo
  
Install the gym-gazebo following the [installation instructions](#installation) given in the repository. 
Note: 
For installation on ROS Indigo:- 
1. Get the following packages individually from their git respository(**indigo branch**) and replace/insert(if not already present) them in the catkin_ws/src/ directory. Run catkin_make after copying these packages.
  a. [ar_track_alvar_msgs](https://github.com/sniekum/ar_track_alvar_msgs)
  b. [ecl_core](https://github.com/stonier/ecl_core/tree/release/0.61-indigo-kinetic)
  c. [ecl_navigation](https://github.com/stonier/ecl_navigation/tree/release/0.60-indigo-kinetic)
  d. [ecl_tools](https://github.com/stonier/ecl_tools/tree/release/0.61-indigo-kinetic)
  e. [ecl_lite](https://github.com/stonier/ecl_lite.git)
  f. [mav_comm](https://github.com/ethz-asl/mav_comm)
Now, for the husky environment we require packages provided by Clearpath Robotics for model import.
  g. [husky](https://github.com/husky/husky)
  h. Copy all the packages given in the additional_packages folder in the path ../gym-gazebo/gym_gazebo/envs/installation/
  
  
2. Get the plugins ready
  a. The plugins for gazebo world allow us to define behaviour in gazebo world(Random pose, Random velocity of obstacles,etc).
  b. Plugins required for our experiment are located in ../gym-gazebo/gym_gazebo/envs/husky/husky_plugin .
    Run the following commands to build the plugin. For more details regarding usage, see the README in the husky_plugin    folder.
    ```mkdir build
    cd build
    cmake ..
    make
    ```
  c. Add the following line to your bashrc.
  ```  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:../gym-gazebo/gym_gazebo/envs/husky/husky_plugin/build
  ```
 

### Running experiments
  ```cd ../gym-gazebo/examples/husky
    python husky__wall_moving_obstacles_lidar_qlearn
  ```



**`gym-gazebo` is a complex piece of software for roboticists that puts together simulation tools, robot middlewares (ROS, ROS 2), machine learning and reinforcement learning techniques. All together to create an environment whereto benchmark and develop behaviors with robots. Setting up `gym-gazebo` appropriately requires relevant familiarity with these tools.**

**Code is available "as it is" and currently it's not supported by any specific organization. Community support is available [here](https://github.com/erlerobot/gym-gazebo/issues). Pull requests and contributions are welcomed.**

-----

## Table of Contents
- [Environments](#community-maintained-environments)
- [Installation](#installation)
- [Usage](#usage)


## Community-maintained environments
The following are some of the gazebo environments maintained by the community using `gym-gazebo`. If you'd like to contribute and maintain an additional environment, submit a Pull Request with the corresponding addition.

| Name | Middleware | Description | Observation Space | Action Space | Reward range |
| ---- | ------ | ----------- | ----- | --------- | -------- |
| ![GazeboCircuit2TurtlebotLidar-v0](imgs/GazeboCircuit2TurtlebotLidar-v0.png)`GazeboCircuit2TurtlebotLidar-v0` | ROS | A simple circuit with straight tracks and 90 degree turns. Highly discretized LIDAR readings are used to train the Turtlebot. Scripts implementing **Q-learning** and **Sarsa** can be found in the _examples_ folder. | | | |
| ![GazeboCircuitTurtlebotLidar-v0](imgs/GazeboCircuitTurtlebotLidar-v0.png)`GazeboCircuitTurtlebotLidar-v0.png` | ROS | A more complex maze  with high contrast colors between the floor and the walls. Lidar is used as an input to train the robot for its navigation in the environment. | | | TBD |
| `GazeboMazeErleRoverLidar-v0` | ROS, [APM](https://github.com/erlerobot/ardupilot) | **Deprecated** | | | |
| `GazeboErleCopterHover-v0` | ROS, [APM](https://github.com/erlerobot/ardupilot) | **Deprecated** | | | |

## Other environments (no support provided for these environments)

The following table compiles a number of other environments that **do not have
community support**.

| Name | Middleware | Description | Observation Space | Action Space | Reward range |
| ---- | ------ | ----------- | ----- | --------- | -------- |
| ![cartpole-v0.png](imgs/cartpole.jpg)`GazeboCartPole-v0` | ROS | | Discrete(4,) | Discrete(2,) | 1) Pole Angle is more than ±12° 2)Cart Position is more than ±2.4 (center of the cart reaches the edge of the display) 3) Episode length is greater than 200 |
| ![GazeboModularArticulatedArm4DOF-v1.png](imgs/GazeboModularArticulatedArm4DOF-v1.jpg)`GazeboModularArticulatedArm4DOF-v1` | ROS | This environment present a modular articulated arm robot with a two finger gripper at its end pointing towards the workspace of the robot.| Box(10,) | Box(3,) | (-1, 1) [`if rmse<5 mm 1 - rmse else reward=-rmse`]|
| ![GazeboModularScara4DOF-v3.png](imgs/GazeboModularScara4DOF-v3.png)`GazeboModularScara4DOF-v3` | ROS | This environment present a modular SCARA robot with a range finder at its end pointing towards the workspace of the robot. The goal of this environment is defined to reach the center of the "O" from the "H-ROS" logo within the workspace. This environment compared to `GazeboModularScara3DOF-v2` is not pausing the Gazebo simulation and is tested in algorithms that solve continuous action space (PPO1 and ACKTR from baselines).This environment uses `slowness=1` and matches the delay between actions/observations to this value (slowness). In other words, actions are taken at "1/slowness" rate.| Box(10,) | Box(3,) | (-1, 1) [`if rmse<5 mm 1 - rmse else reward=-rmse`]|
| ![GazeboModularScara3DOF-v3.png](imgs/GazeboModularScara3DOF-v3.png)`GazeboModularScara3DOF-v3` | ROS | This environment present a modular SCARA robot with a range finder at its end pointing towards the workspace of the robot. The goal of this environment is defined to reach the center of the "O" from the "H-ROS" logo within the workspace. This environment compared to `GazeboModularScara3DOF-v2` is not pausing the Gazebo simulation and is tested in algorithms that solve continuous action space (PPO1 and ACKTR from baselines).This environment uses `slowness=1` and matches the delay between actions/observations to this value (slowness). In other words, actions are taken at "1/slowness" rate.| Box(9,) | Box(3,) | (-1, 1) [`if rmse<5 mm 1 - rmse else reward=-rmse`]|
| ![GazeboModularScara3DOF-v2.png](imgs/GazeboModularScara3DOF-v2.png)`GazeboModularScara3DOF-v2` | ROS | This environment present a modular SCARA robot with a range finder at its end pointing towards the workspace of the robot. The goal of this environment is defined to reach the center of the "O" from the "H-ROS" logo within the workspace. Reset function is implemented in a way that gives the robot 1 second to reach the "initial position".| Box(9,) | Box(3,) | (0, 1) [1 - rmse] |
| ![GazeboModularScara3DOF-v1.png](imgs/GazeboModularScara3DOF-v1.png)`GazeboModularScara3DOF-v1` | ROS | **Deprecated** | | | TBD |
| ![GazeboModularScara3DOF-v0.png](imgs/GazeboModularScara3DOF-v0.png)`GazeboModularScara3DOF-v0` | ROS | **Deprecated** | | | | TBD |
| ![ariac_pick.jpg](imgs/ariac_pick.jpg)`ARIACPick-v0` | ROS | | | |  |

## Installation
Refer to [INSTALL.md](INSTALL.md)

## Usage

### Build and install gym-gazebo

In the root directory of the repository:

```bash
sudo pip install -e .
```

### Running an environment

- Load the environment variables corresponding to the robot you want to launch. E.g. to load the Turtlebot:

```bash
cd gym_gazebo/envs/installation
bash turtlebot_setup.bash
```

Note: all the setup scripts are available in `gym_gazebo/envs/installation`

- Run any of the examples available in `examples/`. E.g.:

```bash
cd examples/scripts_turtlebot
python circuit2_turtlebot_lidar_qlearn.py
```

### Display the simulation

To see what's going on in Gazebo during a simulation, simply run gazebo client:

```bash
gzclient
```

### Display reward plot

Display a graph showing the current reward history by running the following script:

```bash
cd examples/utilities
python display_plot.py
```

HINT: use `--help` flag for more options.

### Killing background processes

Sometimes, after ending or killing the simulation `gzserver` and `rosmaster` stay on the background, make sure you end them before starting new tests.

We recommend creating an alias to kill those processes.

```bash
echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
```
