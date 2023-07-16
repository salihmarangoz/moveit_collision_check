#!/usr/bin/env python

import os
import rospy

REPO_PATH = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
URDF_PATH = REPO_PATH + "/urdf"

if __name__ == '__main__':
    rospy.init_node('save_current_robot_description', anonymous=True)

    try:
        robot_description = rospy.get_param("/robot_description")
        robot_description_semantic = rospy.get_param("/robot_description_semantic")
    except Exception as e:
        rospy.logerr("Parameter not found: " + str(e))
        exit(-1)

    robot_name = input("Enter robot name (e.g. horti): ")
    robot_description_path = URDF_PATH + "/" + robot_name + "_robot_description.urdf"
    robot_description_semantic_path = URDF_PATH + "/" + robot_name + "_robot_description_semantic.urdf"

    with open(robot_description_path, "w") as f:
        f.write(robot_description)
        rospy.loginfo("robot_description is saved to: " + robot_description_path)

    with open(robot_description_semantic_path, "w") as f:
        f.write(robot_description_semantic)
        rospy.loginfo("robot_description_semantic is saved to: " + robot_description_semantic_path)
