#!/usr/bin/env python

import rospy
import numpy as np
from moveit_collision_check.srv import CheckCollision
from moveit_commander.robot import RobotCommander


if __name__ == "__main__":
  rospy.init_node('generate_collision_dataset')
  num_samples = rospy.get_param("~num_samples", 1000)

  rospy.loginfo("Waiting for the check_collision service...")
  rospy.wait_for_service('/moveit_collision_check/check_collision')
  check_collision = rospy.ServiceProxy('/moveit_collision_check/check_collision', CheckCollision)

  robot_commander = RobotCommander(robot_description="robot_description_moveit_collision_check")

  for i1,l1 in enumerate(robot_commander.get_link_names()):
    for i2,l2 in enumerate(robot_commander.get_link_names()):
      if i1>=i2: continue
      if "head_link" in l1: continue
      if "head_link" in l2: continue
      print("<disable_collisions link1=\"{}\" link2=\"{}\" reason=\"Never\"/>".format(l1,l2))