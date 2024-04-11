## Set Up Docker Container for NBV Repository

Run the Docker container. Inside the workspace, create a `src` directory and clone the repositories below:
- [nbv_active](https://github.com/UTNuclearRobotics/active_nbv/tree/ros_noetic)
- [spot_manipulation_moveit](https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/noetic-devel)

```
cd nbv_demo_ws/src
git clone -b ros_noetic --recursive git@github.com:UTNuclearRobotics/active_nbv.git
git clone -b noetic-devel https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/noetic-devel
```

## Running the Demo on Spot

### Terminals on Spot

1. Run `roscore`.

```
source /opt/ros/noetic/setup.bash && roscore
```

1. Connect to the AugRE51 WiFi network and launch the NRG Spot driver from the `spot_manipulation_driver` package. 
```
ssh spot@192.168.11.151
export ROS_MASTER_URI=http://192.168.11.151:11311 && export ROS_IP=192.168.11.151
source catkin_ws/devel/setup.bash 
roslaunch spot_manipulation_driver nrg_spot_driver.launch
```

2. In another terminal, export ROS master and IP, then launch the Tele-Op node from the `teleop_twist_joy` package.
```
ssh spot@192.168.11.151 
export ROS_MASTER_URI=http://192.168.11.151:11311 && export ROS_IP=192.168.11.151
source /opt/ros/noetic/setup.bash
roslaunch teleop_twist_joy teleop.launch config_filepath:=/home/spot/catkin_ws/atk3.config.yaml
```

3. In another terminal, export ROS master and IP, then launch the Spot arm MoveIt node from the [spot_arm_moveit_config](https://github.com/UTNuclearRobotics/nrg_spot_manipulation_moveit/tree/noetic-devel) package.
```
ssh -X spot@192.168.11.151 
export ROS_MASTER_URI=http://192.168.11.151:11311 && export ROS_IP=192.168.11.151
source catkin_ws/devel/setup.bash
roslaunch spot_arm_moveit_config spot_arm_planning_execution.launch
```


### Terminals on Laptop

4. Inside the Docker container, run the below commands. If ROS is correctly configured on the laptop, then the python MoveIt Commander API should pickup the MoveIt node running on Spot (`spot_arm_moveit_config`) and start following the arm motions.
```
export ROS_MASTER_URI=http://192.168.11.151:11311
source nbv_demo_ws/devel/setup.bash
python3 moveit_joint_execute.py
```

The `moveit_joint_execute.py` depends on `moveit_client.py` and some ROS packages in `active_nbv` repository (robot_helpers) and MoveIt Commander itself. The arm poses are pre-computed for different base paths. By default it should be pre-computed for the S-shaped path around the lab (Starting at the red workbench facing Blake's desk and ending at the back of the lab).
