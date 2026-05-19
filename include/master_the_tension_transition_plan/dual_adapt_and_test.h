#ifndef TRAJ_ADAPT_TEST_NODE_H
#define TRAJ_ADAPT_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <master_the_tension_transition_plan/dual_mtc_routing.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

class TrajAdaptTestNode
{
public:
  explicit TrajAdaptTestNode(const rclcpp::NodeOptions& options);
  ~TrajAdaptTestNode() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface() { return node_->get_node_base_interface(); }
  rclcpp::Node::SharedPtr getNode() { return node_; }

  moveit::planning_interface::MoveGroupInterface& getMoveGroup() { return move_group_; }
  moveit_visual_tools::MoveItVisualTools& getVisualTools() { return visual_tools_; }

  // Main entry point for this test node: read → build dual traj → stretch → dump
  void runFromFiles(const std::string& leader_joint_txt,
                    const std::string& follower_joint_txt,
                    const std::string& leader_tcp_txt,
                    const std::string& follower_tcp_txt,
                    const std::vector<double>& stretch_distance,
                    double ik_timeout,
                    const std::filesystem::path& dump_dir);

  void evaluateClearanceForTrajectory(const planning_scene::PlanningSceneConstPtr& base_scene,
                                      const robot_trajectory::RobotTrajectory& traj,
                                      const std::string& object_id);

  void saveClearanceToJson(const std::string& file_path)
  {
    std::ofstream ofs(file_path);
    ofs << traj_clearance_;
    ofs.close();
  }

  // void updatePlanningScene();

  void moveRobotToFirstWaypoint(const robot_trajectory::RobotTrajectory& traj);

  template <typename T>
  bool getROSParam(const std::string& name, T& value) {
    // returns false if parameter not set or wrong type
    return node_->get_parameter(name, value);
  }

  // If you want a "get or default" version:
  template <typename T>
  T getROSParamOr(const std::string& name, const T& default_value) {
    T v = default_value;
    node_->get_parameter(name, v);  // leaves v as default_value if unset
    return v;
  }
  // void loadCustomScene(const std::string &path);

  // Robot group names
  std::string lead_arm_group_name;
  std::string lead_hand_group_name;
  std::string lead_hand_frame;
  std::string lead_base_frame;

  std::string follow_arm_group_name;
  std::string follow_hand_group_name;
  std::string follow_hand_frame;
  std::string follow_base_frame;

  std::string dual_arm_group_name;

private:
  // --- ROS/MoveIt ---
  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit_visual_tools::MoveItVisualTools visual_tools_;

  // --- file parsing ---
  struct JointTxtData {
    std::vector<double> t;                       // seconds
    std::vector<std::vector<double>> q;          // [M][7]
  };

  struct TcpTxtData {
    std::vector<double> t;                       // seconds
    std::vector<Eigen::Matrix4d> T;              // [M]
  };

  static JointTxtData readJointTxt(const std::string& path, size_t dof_expected = 7);
  static TcpTxtData   readTcpTxt(const std::string& path);

  // Build a dual-arm RobotTrajectory waypoint-by-waypoint
  robot_trajectory::RobotTrajectory buildDualRobotTrajectory(const JointTxtData& lead,
                                                            const JointTxtData& follow) const;

  // Your stretcher, rewritten as a free-standing method in this class
  void stretchRobotTrajectoryInPlace(robot_trajectory::RobotTrajectory& robot_trajectory,
                                    double extension_distance,
                                    double ik_timeout);

  // Loads parameters for group/link/frame names and transforms
  void initializeGroups();
  void initializeTransforms(double default_franka_flange_to_tcp_z,
                            double sensone_height,
                            double extend_finger_length);
  void updatePlanningScene();

  void publishDualTraj(
    const std::string& goal_clip_name,
    const robot_trajectory::RobotTrajectory& dual_traj,
    int stage_id=0,
    int id=0);

  void attachCollisionCable(planning_scene::PlanningScenePtr scene,
                                              const std::string& id, 
                                              double length,
                                              double radius,
                                              Eigen::Vector3d vec_in_world,
                                              const std::string& attach_link, 
                                              std::vector<std::string> touch_links,
                                              bool enable_cable_collision)
  {
    moveit_msgs::msg::AttachedCollisionObject attach_msg;
    attach_msg.link_name = attach_link;
    attach_msg.object.header.frame_id = attach_link;
    attach_msg.object.id = id;

    // Add geometry of cable
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.CYLINDER;
    prim.dimensions = {length, radius}; // height (along local Z), radius

    // Step 1: Create pose in TCP frame (cylinder lying along +X, end at origin)
    Eigen::Isometry3d cylinder_pose_tcp = Eigen::Isometry3d::Identity();
    // Convert direction from world frame into attach_link tcp frame
    Eigen::Isometry3d world_to_hand = scene->getFrameTransform(attach_link).inverse();
    Eigen::Vector3d vec_in_hand = world_to_hand.linear() * vec_in_world.normalized();
    // Calculate the rotation of cylinder Z-axis to align with the direction
    Eigen::Quaterniond align_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), vec_in_hand);
    cylinder_pose_tcp.linear() = align_quat.toRotationMatrix();
    // Rotate cylinder Z-axis → X-axis using +90° about Y
    // cylinder_pose_tcp.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    // Translate it so one end sits at TCP
    cylinder_pose_tcp.translation() =  vec_in_hand.normalized() * (0.5*length);
    // Step 2: Transform to `left_panda_hand` frame
    Eigen::Isometry3d cylinder_pose_in_hand = follow_hand_to_tcp_transform_ * cylinder_pose_tcp;
    // Step 3: Convert to geometry_msgs::Pose
    geometry_msgs::msg::Pose pose_msg = tf2::toMsg(cylinder_pose_in_hand);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("traj_adapt_test_node"), "Attach collision object: " << id 
                              << " position: " << pose_msg.position.x << ", " << pose_msg.position.y << ", " << pose_msg.position.z 
                              << " orientation: " << pose_msg.orientation.x << ", " << pose_msg.orientation.y << ", " << pose_msg.orientation.z << ", " << pose_msg.orientation.w);

    attach_msg.object.primitives.push_back(prim);
    attach_msg.object.primitive_poses.push_back(pose_msg);
    attach_msg.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Ignore collision with both grippers
    attach_msg.touch_links = touch_links;

    scene->processAttachedCollisionObjectMsg(attach_msg);

    // add the object but disable cable collision if enable_cable_collision is false
    if (!enable_cable_collision) {
      collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

      bool allow = true;  // always allowed to collide

      // Let dlo_obj collide with everything by default
      acm.setDefaultEntry(id, allow);
      acm.setEntry(id, allow);
    }

    // visualization
    Eigen::Isometry3d pose_in_world = scene->getFrameTransform(attach_link) * cylinder_pose_in_hand;
    // Convert the pose to a geometry_msgs::Pose for visualization
    geometry_msgs::msg::Pose pose_msg_world = tf2::toMsg(pose_in_world);
    visual_tools_.publishCylinder(pose_msg_world, rviz_visual_tools::ORANGE, length, radius);
    visual_tools_.trigger();
  }

  Eigen::Isometry3d lead_hand_to_tcp_transform_  = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d follow_hand_to_tcp_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lead_flange_to_tcp_transform_  = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d follow_flange_to_tcp_transform_ = Eigen::Isometry3d::Identity();

  std::vector<std::string> lead_franka_joint_names_ = {"right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", "right_panda_joint5", "right_panda_joint6", "right_panda_joint7"};
  std::vector<std::string> follow_franka_joint_names_ = {"left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4", "left_panda_joint5", "left_panda_joint6", "left_panda_joint7"};

  nlohmann::json traj_clearance_ = nlohmann::json::object();

  rclcpp::Publisher<moveit_task_constructor_msgs::msg::SubTrajectory>::SharedPtr subtrajectory_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_state_ = msg;
  }

};

#endif // TRAJ_ADAPT_TEST_NODE_H
