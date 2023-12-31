#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
//-----------------------------
#include <moveit_collision_check/CheckCollision.h>

// using global variables.. ugh...
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
planning_scene::PlanningScenePtr planning_scene_;
moveit::core::RobotModelPtr robot_model_;

bool check_collision(moveit_collision_check::CheckCollision::Request  &req, moveit_collision_check::CheckCollision::Response &res)
{
  robot_state::RobotState robot_state(robot_model_);

  if (req.state.name.size() != req.state.position.size())
  {
    ROS_ERROR("Number of joints and number of positions should be equal!");
    return false;
  }

  if (robot_state.getVariableCount() != req.state.name.size())
  {
    ROS_ERROR("Number of joints and number of robot variables should be equal!");
    return false;
  }

  for (int i=0; i<req.state.name.size(); i++) robot_state.setVariablePosition(req.state.name[i], req.state.position[i]);
  robot_state.update();

  // Check collision
  visual_tools_->publishRobotState(robot_state);
  res.collision_state = visual_tools_->checkAndPublishCollision(robot_state, &(*planning_scene_) );
  
  // Check collision distance
  // Copied and modified contents of checkAndPublishCollision. leaving check collision as separated because that also includes visualization.
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.contacts = true;
  c_req.max_contacts = 10;
  c_req.max_contacts_per_pair = 3;
  c_req.verbose = true;
  c_req.distance = true;
  planning_scene_->checkCollision(c_req, c_res, robot_state); // Check for collisions
  res.collision_distance = c_res.distance;

  return true;
}

// Just for debugging. Use the service for normal use.
void jointStatesCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  robot_state::RobotState robot_state(robot_model_);
  for (int i=0; i<msg->name.size(); i++) robot_state.setVariablePosition(msg->name[i], msg->position[i]);
  robot_state.update();
  visual_tools_->checkAndPublishCollision(robot_state, &(*planning_scene_) );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_check");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  bool debug = nhp.param("debug", false);
  //ros::AsyncSpinner spinner(4);
  //spinner.start();

  ros::Subscriber sub;
  if (debug)
  {
    sub = nh.subscribe("/joint_states", 2, jointStatesCallback);
  }

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  robot_model_ = robot_model_loader.getModel();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(), "collision_check_debug"));
  //visual_tools->loadPlanningSceneMonitor();

  ros::ServiceServer service = nhp.advertiseService("check_collision", check_collision);

  //ros::waitForShutdown();
  ros::spin();
}