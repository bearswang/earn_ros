# earn_ros

This is the ROS Wrapper of [EARN]

## Preqrequisite

Please install the following libraries and packages first.

- Python >= 3.8
- [CVXPY](https://www.cvxpy.org/)
- [ROS Noetic](https://wiki.ros.org/noetic)
- [Carla](https://carla.org/)
- [Carla-ROS-bridge](https://github.com/carla-simulator/ros-bridge)

> [!NOTE]
> *We recommend using Python 3.8 and Conda to manage Python environments, as examples are based on CARLA.*

## Test Environment 

- Ubuntu 20.04

## Installation

You can install the package by running the commands below:

```
mkdir -p ~/earn_ws/src
cd ~/earn_ws/src
git clone https://github.com/bearswang/earn_ros
cd ~/earn_ws 
git clone https://github.com/carla-simulator/ros-bridge
cd .. && catkin_make
```
or
```
cd ~/earn_ws/src/earn_ros
bash ./setup.sh
```

## Demonstration

### Autonomous Driving

We provide the Autonomous Driving example in Carla. 
You can try the demo by running the commands below:

```
cd $CARLA_ROOT
bash ./CarlaUE4.sh
cd ~/earn_ws
source devel/setup.bash
roslaunch earn_ros Town04_spawn_earn.launch
roslaunch earn_ros Town04_traffic_earn.launch 
roslaunch earn_ros Town04_run_earn.launch 
```
or
```
cd $CARLA_ROOT
bash ./CarlaUE4.sh
cd ~/earn_ws/src/earn_ros
bash ./earn_demo.sh
```

> [!NOTE]
> CARLA_ROOT is the roof folder that installs CARLA

Then you will see Edge Accelerated Navigation in Town04 with traffics:



https://github.com/user-attachments/assets/0fb08cbe-ae02-4837-97a1-22db647f6c43



## Node API


### Parameters

#### Robot Information

| Parameter               | Type             | Default Value | Description                                                                                          |
| ----------------------- | ---------------- | ------------- | ---------------------------------------------------------------------------------------------------- |
| `~robot_info`           | `dict`           |               | Configuration dictionary for robot specifications.                                                   |
| `~robot_info.vertices`  | `list` or `None` | `None`        | Vertices defining the robot's shape. If `None`, uses length and width to create a rectangular shape. |
| `~robot_info.radius`    | `float`          | `None`        | Radius of the robot (used if the robot shape is circular).                                           |
| `~robot_info.max_speed` | `list`           | `[5, 1]`     | Maximum speed parameters `[linear, angular]` or `[linear, steer]`.                                   |
| `~robot_info.max_acce`  | `list`           | `[1, 0.5]`   | Maximum acceleration parameters `[linear, angular]` or `[linear, steer]`.                            |
| `~robot_info.length`    | `float`          | `4.69`           | Length of the robot (used if vertices are not provided).                                             |
| `~robot_info.width`     | `float`          | `1.85`           | Width of the robot (used if vertices are not provided).                                              |
| `~robot_info.wheelbase` | `float`          | `2.87`         | Wheelbase of the robot.                                                                              |
| `~robot_info.dynamics`  | `string`         | `"acker"`      | Type of robot dynamics (e.g., differential: `diff`, ackermann: `acker`).                             |
| `~robot_info.cone_type` | `string`         | `"Rpositive"` | Type of collision cone used, polygon: `Rpositive`, circle: `norm2`                                   |

#### PDD Configuration

| Parameter               | Type    | Default Value | Description                                                |
| ----------------------- | ------- | ------------- | ---------------------------------------------------------- |
| `~receding`             | `int`   | `15`          | Receding horizon parameter for MPC.                        |
| `~iter_num`             | `int`   | `3`           | Number of iterations for the MPC solver.                   |
| `~enable_reverse`       | `bool`  | `False`       | Enables reverse movement if set to `True`.                 |
| `~sample_time`          | `float` | `0.3`         | Sampling time interval for the MPC.                        |
| `~process_num`          | `int`   | `4`           | Number of parallel processes for MPC computation.          |
| `~accelerated`          | `bool`  | `True`        | Enables accelerated computation in MPC.                    |
| `~time_print`           | `bool`  | `False`       | Enables time logging for MPC operations.                   |
| `~obstacle_order`       | `bool`  | `True`        | Determines if obstacle ordering by distance is applied.    |
| `~max_edge_num`         | `int`   | `4`           | Maximum number of edges to consider for obstacles.         |
| `~max_obs_num`          | `int`   | `4`           | Maximum number of obstacles to consider.                   |
| `~goal_index_threshold` | `int`   | `1`           | Threshold for goal index determination.                    |
| `~iter_threshold`       | `float` | `0.2`         | Iteration threshold for convergence in MPC.                |
| `~slack_gain`           | `float` | `10`           | Gain for slack variables in MPC constraints.               |
| `~max_sd`               | `float` | `1.0`         | Maximum safety distance.                                   |
| `~min_sd`               | `float` | `0.1`         | Minimum safety distance.                                   |
| `~ws`                   | `float` | `0.2`         | Weight for the state in the cost function.                 |
| `~wu`                   | `float` | `1.4`         | Weight for the control inputs in the cost function.        |
| `~ro1`                  | `float` | `0.1`         | Weight parameter for the first term in the cost function.  |
| `~ro2`                  | `float` | `0.1`         | Weight parameter for the second term in the cost function. |
| `~pdd_en`               | `bool` | `True`         | Enables penalty dual decomposition.  |
| `~edge_accelerate`      | `bool` | `False`         | Enables edge collaboration.  |


#### MPS Configuration

| Parameter               | Type    | Default Value | Description                                                |
| ----------------------- | ------- | ------------- | ---------------------------------------------------------- |
| `~edge_position`             | `list`   | `[255, 172]`          | Position of edge server.                        |
| `~coverage`             | `int`   | `500`          | Communication coverage of edge server.                        |
| `~Cth`             | `int`   | `30`           | Maximum computation load in ms for the edge solver.                   |
| `~receding`             | `int`   | `15`          | Receding horizon parameter for MPC.                        |
| `~max_edge_num`         | `int`   | `4`           | Maximum number of edges to consider for obstacles.         |
| `~max_obs_num`          | `int`   | `4`           | Maximum number of obstacles to consider.                   |

#### Reference Speed

| Parameter    | Type    | Default Value | Description                    |
| ------------ | ------- | ------------- | ------------------------------ |
| `~ref_speed` | `float` | `4.5`         | Reference speed for the robot. |


> [!NOTE]
Earn_ros is developed based on RDA-planner and rda_ros. 
You may want to see more information in [RDA-planner](https://github.com/hanruihua/RDA-planner) and [rda_ros](https://github.com/hanruihua/rda_ros)

## Citation

If you find this code or paper helpful, please consider **starring** the repository and **citing** our work using the following BibTeX entries:

```
@ARTICLE{10601554,
  author={Li, Guoliang and Han, Ruihua and Wang, Shuai and Gao, Fei and Eldar, Yonina C. and Xu, Chengzhong},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Edge Accelerated Robot Navigation With Collaborative Motion Planning}, 
  year={2025},
  volume={30},
  number={2},
  pages={1166-1178},
  doi={10.1109/TMECH.2024.3419436}}
```

```
  @ARTICLE{10036019,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Zhang, Qianru and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Robotics and Automation Letters}, 
  title={RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments}, 
  year={2023},
  volume={8},
  number={3},
  pages={1715-1722},
  doi={10.1109/LRA.2023.3242138}}

```

## Acknowledgement

- [RDA-planner](https://github.com/hanruihua/RDA-planner)
- [rda_ros](https://github.com/hanruihua/rda_ros)
- [Carla](https://carla.org/)
- [Carla-ROS-bridge](https://github.com/carla-simulator/ros-bridge)

## Authors

[Shuai Wang](https://github.com/bearswang)

[Guoliang Li](https://github.com/GuoliangLI1998)
