# Setting up Repository

Follow the steps in this [active_nbv](https://github.com/UTNuclearRobotics/active_nbv/tree/ros_noetic) repository.

# Running the Demo on Spot

## Terminals on Spot

1. Connect to the AugRE51 WiFi network and launch the NRG Spot driver from the `spot_manipulation_driver` package. 
```
ssh spot@192.168.11.151 
cd catkin_ws 
source devel/setup.bash 
export ROS_IP=192.168.11.151 
roslaunch spot_manipulation_driver nrg_spot_driver.launch
```

2. In another terminal, export ROS master and IP, then launch the Tele-Op node from the `teleop_twist_joy` package.
```
ssh spot@192.168.11.151 
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.151
roslaunch teleop_twist_joy teleop.launch config_filepath:=/home/spot/catkin_ws/atk3.config.yaml
```

3. In another terminal, export ROS master and IP, then launch the Spot arm MoveIt node from the [spot_arm_moveit_config](https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/nrg_devel) package.
```
ssh spot@192.168.11.151 
export ROS_MASTER_URI=http://192.168.11.151:11311
export ROS_IP=192.168.11.151
roslaunch spot_arm_moveit_config spot_arm_planning_execution.launch
```


## Terminals on Laptop

4. If ROS is correctly configured on the laptop, then the python MoveIt Commander API should pickup the MoveIt node running on Spot (`spot_arm_moveit_config`) and start following the arm motions.
```
cd nbv_demo_ws
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.11.151:11311
python3 moveit_joint_execute.py
```

The `moveit_joint_execute.py` depends on `moveit_client.py` and some ROS packages in `active_nbv` repository (robot_helpers) and MoveIt Commander itself. The arm poses are pre-computed for different base paths. By default it should be pre-computed for the S-shaped path around the lab (Starting at the red workbench facing Blake's desk and ending at the back of the lab).