#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
#include <fstream>
#include <string>

float positionx;
float positiony = 0.6;
float positionz = 0.2;
float zcal;
float ycal;
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.2;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.255;
  collision_objects[0].primitive_poses[0].position.z = -0.21;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void addCeiling(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "ceiling";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.2;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.255;
  collision_objects[0].primitive_poses[0].position.z = 0.965;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);

}
void addWalls(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "wall1";
  collision_objects[1].id = "wall2";
  collision_objects[2].id = "wall3";
  collision_objects[3].id = "wall4";
  collision_objects[0].header.frame_id = "base_link";
  collision_objects[1].header.frame_id = "base_link";
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[3].header.frame_id = "base_link";

  /*Wall 1*/
  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.03;
  collision_objects[0].primitives[0].dimensions[1] = 1.2;
  collision_objects[0].primitives[0].dimensions[2] = 0.94;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.585;
  collision_objects[0].primitive_poses[0].position.y = 0.255;
  collision_objects[0].primitive_poses[0].position.z = 0.45;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  /*Wall 2*/
  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1.2;
  collision_objects[1].primitives[0].dimensions[1] = 0.03;
  collision_objects[1].primitives[0].dimensions[2] = 0.94;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = -0.330;
  collision_objects[1].primitive_poses[0].position.z = 0.45;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  /*Wall 3*/
  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.03;
  collision_objects[2].primitives[0].dimensions[1] = 1.2;
  collision_objects[2].primitives[0].dimensions[2] = 0.94;

  /* Define the pose of the table. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.585;
  collision_objects[2].primitive_poses[0].position.y = 0.255;
  collision_objects[2].primitive_poses[0].position.z = 0.45;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL


  planning_scene_interface.applyCollisionObjects(collision_objects);

}


void moveEE(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion orientation;
  float pitch;
  float yaw;
  pitch = 3.14;
  yaw = 1.5707;
  orientation.setRPY(-1.5707, -pitch/2, -yaw);
  //orientation.setRPY(-M_PI *1/2, 0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  int randomNumber = std::rand() % 11;
  randomNumber = randomNumber + 8;
  float floatrandom = randomNumber;
  std::cout << floatrandom/100 << std::endl;
  target_pose.position.x = floatrandom/100;
  target_pose.position.y = 0.575;
  target_pose.position.z = 0.2;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);
  //move_group_interface_arm.setMaxVelocityScalingFactor(0.5);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
  goal_plan.trajectory_ = trajectory;
  move_group_interface.execute(goal_plan);
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
}

void moveEE2(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion orientation;
  float pitch;
  float yaw;
  pitch = 3.14;
  yaw = 1.5707;
  //orientation.setRPY(-1.5707, -pitch/2, -yaw);
  //orientation.setRPY(-M_PI *1/2, 0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = 0;
  target_pose.position.y = 0.5;
  target_pose.position.z = 0.0;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);
  //move_group_interface_arm.setMaxVelocityScalingFactor(0.5);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
  goal_plan.trajectory_ = trajectory;
  move_group_interface.execute(goal_plan);
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
}

void moveEE3(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion orientation;
  float pitch;
  float yaw;
  pitch = 1.5707;
  yaw = 1.5707;
  orientation.setRPY(0, -pitch, -yaw);
  //orientation.setRPY(-M_PI *1/2, 0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = -0.2;
  target_pose.position.y = 0.4;
  target_pose.position.z = 0.6;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);
  //move_group_interface_arm.setMaxVelocityScalingFactor(0.5);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
  goal_plan.trajectory_ = trajectory;
  move_group_interface.execute(goal_plan);
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
}

void moveEE4(moveit::planning_interface::MoveGroupInterface& move_group_interface)
{
  std::ifstream inputFile("/home/theodore/ros2_ws/src/moveit_nodes/moveit_nodes/mypipe.txt");
  std::string line;
  char ptr;
  while (std::getline(inputFile, line)) {
      // Process the line here
      std::string delimiter = ",";
      std::vector<std::string> tokens;
      size_t pos = 0;

      while ((pos = line.find(delimiter)) != std::string::npos) {
          tokens.push_back(line.substr(0, pos));
          line.erase(0, pos + delimiter.length());
      }

      tokens.push_back(line); // Add the last token

      zcal = - 0.02 - (150 - stoi(tokens[1]) -5)*(0.012/(stoi(tokens[3])-stoi(tokens[1])));
      if (stoi(tokens[0]) >150)
      {
        ycal = - 0.0885 + (150 - stoi(tokens[0]) - 8)*(0.012/(stoi(tokens[3])-stoi(tokens[1])));  
      }
      else
      {
        ycal = - 0.0885 + (150 - stoi(tokens[0]) - 8)*(0.012/(stoi(tokens[3])-stoi(tokens[1])));
      }
      std::cout << stoi(tokens[0]) << std::endl;
      std::cout << zcal << std::endl;
      std::cout << ycal << std::endl;
  }


  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion orientation;
  float pitch;
  float yaw;
  pitch = 3.14;
  yaw = 1.5707;
  
  //zcal = 0.03502622951;
  //ycal = 0.0006557377;
  orientation.setRPY(-1.5707, -pitch/2, -yaw);
  //orientation.setRPY(-M_PI *1/2, 0, 0);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = 0.17;
  target_pose.position.y = positiony + ycal;
  target_pose.position.z = positionz + zcal;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);
  //move_group_interface_arm.setMaxVelocityScalingFactor(0.5);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
  goal_plan.trajectory_ = trajectory;
  move_group_interface.execute(goal_plan);
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  
  using moveit::planning_interface::MoveGroupInterface;
  //auto move_group_interface = MoveGroupInterface(node, "ur5e_arm");
  //auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //addCollisionObjects(planning_scene_interface);
  //addCeiling(planning_scene_interface);
  //addWalls(planning_scene_interface);
  moveEE2(move_group_interface);
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //moveEE(move_group_interface);
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //moveEE4(move_group_interface);
  
  // Execute the plan
  /*if (success)
  { std::this_thread::sleep_for(std::chrono::seconds(2));
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  //moveEE(move_group_interface);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  moveEE2(move_group_interface);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  moveEE3(move_group_interface);*/
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

