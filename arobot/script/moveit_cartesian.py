#!/usr/bin/env python3
import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
from geometry_msgs.msg import Pose

# This script plans a straight line path from the current robot pose to pose_goal
# This plan can then be executed on the robot using the execute_plan method

# create a MoveItPlanner object and start the moveit node
franka_moveit = MoveItPlanner()

# Construct the Pose goal in panda_end_effector frame (that you read from fa.get_pose())
pose_goal = Pose()
pose_goal.position.x = 0.42819182
pose_goal.position.y = 0.22212621
pose_goal.position.z = 0.21289236
pose_goal.orientation.x = 1
pose_goal.orientation.y = 0
pose_goal.orientation.z = 0
pose_goal.orientation.w = 0


# convert pose goal to the panda_hand frame (the frame that moveit uses)
pose_goal_moveit = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)

# plan a straight line motion to the goal
plan = franka_moveit.get_straight_plan_given_pose(pose_goal_moveit)

# execute the plan (uncomment after verifying plan on rviz)
franka_moveit.execute_plan(plan)