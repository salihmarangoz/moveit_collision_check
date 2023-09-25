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
  robot_state = robot_commander.get_current_state()

  for i in range(num_samples):
    # randomize robot state
    new_position = []
    for idx, joint_name in enumerate(robot_state.joint_state.name):
      joint = robot_commander.get_joint(joint_name)
      new_position.append( np.random.uniform(joint.min_bound(), joint.max_bound()) )
    robot_state.joint_state.position = new_position

    res = check_collision(robot_state.joint_state)
    print(res)
    #rospy.sleep(1.0)