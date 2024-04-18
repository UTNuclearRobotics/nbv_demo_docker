import moveit_commander
# from moveit_msgs.msg import CollisionObject
import numpy as np

from robot_helpers.ros.conversions import to_mesh_msg, to_pose_msg
from robot_helpers.spatial import Transform


class MoveItClient:
    def __init__(self, planning_group):
        self.planning_group = planning_group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)

    def goto(self, target, velocity_scaling=0.1, acceleration_scaling=0.1):
        _, plan = self.plan(target, velocity_scaling, acceleration_scaling)
        success = self.execute(plan)
        return success

    def plan(self, target, velocity_scaling=0.1, acceleration_scaling=0.1):
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)

        if isinstance(target, Transform):
            self.move_group.set_pose_target(to_pose_msg(target))
        elif isinstance(target, (list, np.ndarray)):
            self.move_group.set_joint_value_target(target)
        elif isinstance(target, str):
            self.move_group.set_named_target(target)
        else:
            raise ValueError

        success, plan, _, _ = self.move_group.plan()
        return success, plan

    def gotoL(self, target, velocity_scaling=0.1, acceleration_scaling=0.1):
        plan, fraction = self.planL(target, velocity_scaling, acceleration_scaling)
        success = self.execute(plan)
        return success, fraction

    def planL(self, target, velocity_scaling=0.1, acceleration_scaling=0.1):
        waypoints = [to_pose_msg(target)]
        # plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.05, 0.5)
        state = self.robot.get_current_state()
        return self.move_group.retime_trajectory(
            state,
            plan,
            velocity_scaling_factor=velocity_scaling,
            acceleration_scaling_factor=acceleration_scaling,
            algorithm="time_optimal_trajectory_generation",), fraction
    
    def gotoLs(self, targets, velocity_scaling=0.1, acceleration_scaling=0.1):
        plan, fraction = self.planLs(targets, velocity_scaling, acceleration_scaling)
        success = self.execute(plan)
        return success, fraction
    
    def planLs(self, targets, velocity_scaling=1, acceleration_scaling=1):

        waypoints = [to_pose_msg(x) for x in targets]
        # plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        fraction = 0
        self.move_group.set_max_velocity_scaling_factor(0.625)
        self.move_group.set_max_acceleration_scaling_factor(0.625)
        while fraction<0.95:
            plan, fraction = self.move_group.compute_cartesian_path(waypoints,  eef_step=1., jump_threshold=5, avoid_collisions=True, path_constraints=None ) #\
                                                                    # ,max_velocity_scaling_factor=velocity_scaling, max_acceleration_scaling_factor=acceleration_scaling,)
        state = self.robot.get_current_state()
        return self.move_group.retime_trajectory(
            state,
            plan,
            velocity_scaling_factor=velocity_scaling,
            acceleration_scaling_factor=acceleration_scaling,
            algorithm="time_optimal_trajectory_generation",), fraction

    def execute(self, plan):
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success