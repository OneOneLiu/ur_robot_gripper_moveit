#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Setup MoveGroupInterface and PlanningSceneInterface
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Visualization tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial", move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // Define a box collision object
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = move_group.getPlanningFrame();
  box.id = "box1";

  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions = {0.1, 0.1, 0.1};  // Dimensions of the box

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;

  box.primitives.push_back(box_primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  // Add the box to the planning scene
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add the box to the scene");
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {box};
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Define a cylinder collision object to attach
  moveit_msgs::msg::CollisionObject cylinder;
  cylinder.id = "cylinder1";
  cylinder.header.frame_id = move_group.getEndEffectorLink();

  shape_msgs::msg::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = cylinder_primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.2;
  cylinder_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.04;

  // Define the pose of the cylinder relative to the end effector
  geometry_msgs::msg::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.z = 0.1;  // Position it at the gripper's location

  cylinder.primitives.push_back(cylinder_primitive);
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.operation = cylinder.ADD;

  // Attach the cylinder to the robot
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add the cylinder to the scene");
  planning_scene_interface.applyCollisionObject(cylinder); // apply用于添加单个物体，add用于添加多个物体

  // Attach the cylinder to the robot
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to attach the cylinder to the robot");

  std::vector<std::string> touch_links = {"robotiq_85_right_finger_tip_link", "robotiq_85_left_finger_tip_link"};
  move_group.attachObject(cylinder.id, move_group.getEndEffectorLink(), touch_links);

  // visual_tools.publishText(Eigen::Isometry3d::Identity(), "Cylinder Attached to Robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to detach the cylinder");

  RCLCPP_INFO(LOGGER, "Detach the object from the robot"); move_group.detachObject(cylinder.id);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to remove the objects");
  std::vector<std::string> object_ids;
  object_ids.push_back(box.id);
  object_ids.push_back(cylinder.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Wait for confirmation that objects are removed
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to confirm objects are removed");

  rclcpp::shutdown();
  return 0;
}
