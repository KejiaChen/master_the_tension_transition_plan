#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <moveit_task_constructor_msgs/srv/get_clip_names.hpp>
#include <moveit/task_constructor/storage.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <vector>

#include <master_the_tension_transition_plan/dual_adapt_and_test.h>
#include <moveit/task_constructor/stages/noop.h>
#include <moveit/task_constructor/cost_terms.h>
// #include <moveit/task_constructor/task_routing.h>
// #include <moveit_task_constructor_msgs/msg/clip_names.hpp>
// #include <moveit_msgs>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

using boost::asio::ip::udp;
using json = nlohmann::json;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("traj_adapt_test_node");

TrajAdaptTestNode::TrajAdaptTestNode(const rclcpp::NodeOptions& options)
: node_(std::make_shared<rclcpp::Node>("traj_adapt_test_node", options)),
  robot_model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description")),
  robot_model_(robot_model_loader_->getModel()),
  // MoveGroupInterface needs a planning group; we’ll set it after params load
  move_group_(node_, "dual_arm"),
  visual_tools_(node_, "world")
{
  if (!robot_model_) {
    throw std::runtime_error("Failed to load RobotModel from robot_description");
  }

  subtrajectory_publisher_ = node_->create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
		    "/mtc_sub_trajectory", rclcpp::QoS(1).transient_local());

  // Subscription to the joint states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&TrajAdaptTestNode::jointStateCallback, this, std::placeholders::_1));

  // Wait until joint states are received
  RCLCPP_INFO(LOGGER, "Waiting for joint states...");
  while (!current_joint_state_)
  {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(LOGGER, "Joint states received.");

  initializeTransforms(/*default_franka_flange_to_tcp_z*/ 0.1034,
                        /*sensone_height*/ 0.036,
                        /*extend_finger_length*/ 0.01);

  initializeGroups();

  visual_tools_.loadRemoteControl();
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();
}

void TrajAdaptTestNode::initializeGroups()
{
  // Define robot group names
  lead_arm_group_name = "right_panda_arm";
  lead_hand_group_name = "right_hand";
  lead_hand_frame = "right_panda_hand";
  lead_base_frame = "right_panda_link0";

  follow_arm_group_name = "left_panda_arm";
  follow_hand_group_name = "left_hand";
  follow_hand_frame = "left_panda_hand";
  follow_base_frame = "left_panda_link0";

  dual_arm_group_name = "dual_arm";
}

void TrajAdaptTestNode::initializeTransforms(double default_franka_flange_to_tcp_z,
                          double sensone_height,
                          double extend_finger_length)
{
  // CAUTION: flange_to_tcp stands for the transform from panda_link_8 to TCP
  // In comparison to hand_to_tcp, there is additional rotation of 45 degree around z axis, and an optional z offset because of wrist snesor
  // adapt flange to TCP transform based on the wrist sensor's height
  follow_flange_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z; 
  if (node_->get_parameter("use_sensone_left").as_bool()){
    follow_flange_to_tcp_transform_.translation().z() += sensone_height;
  }
  // set rotation to 45 degree around z axis
  follow_flange_to_tcp_transform_.rotate(Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ())); // link8 rotates 45 degree around z axis to tcp
  RCLCPP_INFO(LOGGER, "Follower flange to TCP transform z: %f", follow_flange_to_tcp_transform_.translation().z());
  
  lead_flange_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z; 
  if (node_->get_parameter("use_sensone_right").as_bool()){
    lead_flange_to_tcp_transform_.translation().z() += sensone_height; 
  }
  lead_flange_to_tcp_transform_.rotate(Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ())); // link8 rotates 45 degree around z axis to tcp
  RCLCPP_INFO(LOGGER, "Leader flange to TCP transform z: %f", lead_flange_to_tcp_transform_.translation().z());
  
  // hand_to_TCP transform is different from flange_to_TCP transform, it is usually a fixed value if franka hand is not changed
  follow_hand_to_tcp_transform_ = Eigen::Isometry3d::Identity();
  follow_hand_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z; // 0.1034 is the default value for panda hand
  if (node_->get_parameter("alter_finger_left").as_bool()){
    follow_hand_to_tcp_transform_.translation().z() += extend_finger_length*0.5;
    follow_flange_to_tcp_transform_.translation().z() += extend_finger_length*0.5; // 0.1034 is the default value for panda flange
    RCLCPP_INFO(LOGGER, "Altered follower hand to TCP transform z: %f", follow_hand_to_tcp_transform_.translation().z());
  }else{
    RCLCPP_INFO(LOGGER, "Default follower hand to TCP transform z: %f", follow_hand_to_tcp_transform_.translation().z());
  }

  lead_hand_to_tcp_transform_ = Eigen::Isometry3d::Identity();
  lead_hand_to_tcp_transform_.translation().z() = default_franka_flange_to_tcp_z; // 0.1034 is the default value for panda hand
  if (node_->get_parameter("alter_finger_right").as_bool()){
    lead_hand_to_tcp_transform_.translation().z() += extend_finger_length*0.5; // 0.1034 is the default value for panda hand
    lead_flange_to_tcp_transform_.translation().z() += extend_finger_length*0.5; // 0.1034 is the default value for panda flange
    RCLCPP_INFO(LOGGER, "Altered leader hand to TCP transform z: %f", lead_hand_to_tcp_transform_.translation().z());
  }else{
    RCLCPP_INFO(LOGGER, "Default leader hand to TCP transform z: %f", lead_hand_to_tcp_transform_.translation().z());
  }

}

void TrajAdaptTestNode::publishDualTraj(
    const std::string& goal_clip_name,
    const robot_trajectory::RobotTrajectory& dual_traj,
    int stage_id /*= 0*/,
    int id /*= 0*/)
{
  if (dual_traj.getWayPointCount() == 0) {
    RCLCPP_WARN(LOGGER, "publishDualTraj: dual_traj has 0 waypoints");
    return;
  }

  // Convert RobotTrajectory -> ROS msg
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  dual_traj.getRobotTrajectoryMsg(robot_trajectory_msg);

  if (robot_trajectory_msg.joint_trajectory.points.empty()) {
    RCLCPP_WARN(LOGGER, "publishDualTraj: joint_trajectory has 0 points");
    return;
  }

  // ---- Visualize leader & follower exactly like before ----
  // visual_tools_.publishTrajectoryLine(
  //     robot_trajectory_msg,
  //     move_group_.getCurrentState()->getJointModelGroup(lead_arm_group_name),
  //     lead_flange_to_tcp_transform_,
  //     goal_clip_name,
  //     stage_id,
  //     id,
  //     "/home/tp2/ws_humble/trajectories_leader",
  //     rviz_visual_tools::ORANGE,
  //     lead_base_frame);

  // visual_tools_.publishTrajectoryLine(
  //     robot_trajectory_msg,
  //     move_group_.getCurrentState()->getJointModelGroup(follow_arm_group_name),
  //     follow_flange_to_tcp_transform_,
  //     goal_clip_name,
  //     stage_id,
  //     id,
  //     "/home/tp2/ws_humble/trajectories_follower",
  //     rviz_visual_tools::BLUE,
  //     follow_base_frame);

  // visual_tools_.trigger();

  // ---- Publish as ONE SubTrajectory (so your existing subscriber still works) ----
  moveit_task_constructor_msgs::msg::SubTrajectory out;
  out.info.id = id;
  out.info.stage_id = stage_id;
  out.info.cost = 0.0;  // optional; no MTC cost available

  out.trajectory.joint_trajectory = robot_trajectory_msg.joint_trajectory;

  subtrajectory_publisher_->publish(out);

  // ---- Logs (same style as before) ----
  RCLCPP_INFO(LOGGER, "Dual trajectory includes the following joints:");
  for (const auto& joint_name : out.trajectory.joint_trajectory.joint_names)
    RCLCPP_INFO(LOGGER, "  %s", joint_name.c_str());

  RCLCPP_INFO_STREAM(LOGGER,
      "Published dual trajectory as SubTrajectory id " << out.info.id
      << " stage " << out.info.stage_id
      << " with " << out.trajectory.joint_trajectory.points.size()
      << " waypoints");

  visual_tools_.prompt("[Publishing] Press 'next' to continue");
  rclcpp::sleep_for(std::chrono::milliseconds(100));
}


// -------------------- File readers --------------------

TrajAdaptTestNode::JointTxtData TrajAdaptTestNode::readJointTxt(const std::string& path, size_t dof_expected)
{
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open joint txt: " + path);

  JointTxtData out;
  std::string line;

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (line[0] == '#') continue;

    std::istringstream ss(line);
    double t;
    if (!(ss >> t)) continue;

    std::vector<double> q(dof_expected);
    for (size_t i=0; i<dof_expected; ++i) {
      if (!(ss >> q[i])) {
        throw std::runtime_error("Joint txt parse error (missing q) in: " + path);
      }
    }

    // skip velocities if present (dof_expected values)
    for (size_t i=0; i<dof_expected; ++i) {
      double dummy;
      if (!(ss >> dummy)) break; // velocities might be absent; that's ok
    }

    out.t.push_back(t);
    out.q.push_back(std::move(q));
  }

  if (out.t.empty())
    throw std::runtime_error("Joint txt is empty: " + path);

  return out;
}

TrajAdaptTestNode::TcpTxtData TrajAdaptTestNode::readTcpTxt(const std::string& path)
{
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot open tcp txt: " + path);

  TcpTxtData out;
  std::string line;

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (line[0] == '#') continue;

    std::istringstream ss(line);
    double t;
    if (!(ss >> t)) continue;

    Eigen::Matrix4d m;
    for (int col=0; col<4; ++col) {
      for (int row=0; row<4; ++row) {
        if (!(ss >> m(row, col)))
          throw std::runtime_error("TCP txt parse error in: " + path);
      }
    }

    out.t.push_back(t);
    out.T.push_back(m);
  }

  if (out.t.empty())
    throw std::runtime_error("TCP txt is empty: " + path);

  return out;
}

// -------------------- Build dual trajectory --------------------

robot_trajectory::RobotTrajectory TrajAdaptTestNode::buildDualRobotTrajectory(
    const JointTxtData& lead,
    const JointTxtData& follow) const
{
  const size_t M = std::min(lead.t.size(), follow.t.size());
  if (M == 0) throw std::runtime_error("No overlapping waypoints between leader and follower");

  // Use the dual group name here (important for IK, state layout, etc.)
  robot_trajectory::RobotTrajectory traj(robot_model_, dual_arm_group_name);

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();

  // Get joint variable names from groups (we’ll set positions by variable name)
  const auto* lead_jmg   = robot_model_->getJointModelGroup(lead_arm_group_name);
  const auto* follow_jmg = robot_model_->getJointModelGroup(follow_arm_group_name);
  if (!lead_jmg || !follow_jmg)
    throw std::runtime_error("JointModelGroup not found in buildDualRobotTrajectory");

  const auto& lead_names   = lead_jmg->getVariableNames();   // size 7
  const auto& follow_names = follow_jmg->getVariableNames(); // size 7

  auto dt = [&](size_t i)->double{
    if (i == 0) return 0.0;
    const double t_now  = std::min(lead.t[i], follow.t[i]);
    const double t_prev = std::min(lead.t[i-1], follow.t[i-1]);
    return std::max(0.0, t_now - t_prev);
  };

  for (size_t i=0; i<M; ++i) {
    // set leader joints
    for (size_t j=0; j<lead_names.size(); ++j)
      state.setVariablePosition(lead_names[j], lead.q[i][j]);

    // set follower joints
    for (size_t j=0; j<follow_names.size(); ++j)
      state.setVariablePosition(follow_names[j], follow.q[i][j]);

    state.update();

    // IMPORTANT: zero velocities (and optionally accelerations)
    std::vector<double> zeros(robot_model_->getVariableCount(), 0.0);
    state.setVariableVelocities(zeros);

    traj.addSuffixWayPoint(state, dt(i));
  }

  return traj;
}

// -------------------- Stretch logic (rebuild trajectory) --------------------

void TrajAdaptTestNode::stretchRobotTrajectoryInPlace(
    robot_trajectory::RobotTrajectory& robot_trajectory,
    double extension_distance,
    double ik_timeout)
{
  const moveit::core::RobotModelConstPtr& model = robot_trajectory.getRobotModel();
  if (!model)
    throw std::runtime_error("RobotTrajectory has null RobotModel");

  const auto* lead_jmg   = model->getJointModelGroup(lead_arm_group_name);
  const auto* follow_jmg = model->getJointModelGroup(follow_arm_group_name);
  if (!lead_jmg || !follow_jmg)
    throw std::runtime_error("JointModelGroup not found");

  const size_t M = robot_trajectory.getWayPointCount();
  if (M == 0) return;

  robot_trajectory::RobotTrajectory new_traj(model, robot_trajectory.getGroupName());

  // dt fallback (works even if your RobotTrajectory lacks duration helpers)
  auto dt = [&](size_t i)->double{
    if (i == 0) return 0.0;
    // If available in your branch, you can use:
    // return robot_trajectory.getWayPointDurationFromPrevious(i);
    // Otherwise: fixed dt or infer elsewhere. Here we infer from start durations if present.
    double t_i = 0.0, t_im1 = 0.0;
    try {
      t_i   = robot_trajectory.getWayPointDurationFromStart(i);
      t_im1 = robot_trajectory.getWayPointDurationFromStart(i-1);
      return std::max(0.0, t_i - t_im1);
    } catch (...) {
      return 0.05; // fallback
    }
  };

  for (size_t i=0; i<M; ++i)
  {
    moveit::core::RobotState state = robot_trajectory.getWayPoint(i);

    const Eigen::Isometry3d T_lead_hand   = state.getGlobalLinkTransform(lead_hand_frame);
    const Eigen::Isometry3d T_follow_hand = state.getGlobalLinkTransform(follow_hand_frame);

    const Eigen::Isometry3d T_lead_tcp   = T_lead_hand   * lead_hand_to_tcp_transform_;
    const Eigen::Isometry3d T_follow_tcp = T_follow_hand * follow_hand_to_tcp_transform_;

    Eigen::Vector3d dir = safeNormalize(T_lead_tcp.translation() - T_follow_tcp.translation());
    if (!dir.isZero())
    {
      Eigen::Isometry3d T_lead_tcp_des = T_lead_tcp;
      T_lead_tcp_des.translation() += extension_distance * dir;

      const Eigen::Isometry3d T_lead_hand_des =
          T_lead_tcp_des * lead_hand_to_tcp_transform_.inverse();

      (void)state.setFromIK(lead_jmg, T_lead_hand_des, lead_hand_frame, ik_timeout);
    }

    // IMPORTANT: zero velocities (and optionally accelerations)
    std::vector<double> zeros(model->getVariableCount(), 0.0);
    state.setVariableVelocities(zeros);
    new_traj.addSuffixWayPoint(state, dt(i));
  }

  robot_trajectory = new_traj;
}

void TrajAdaptTestNode::moveRobotToFirstWaypoint(const robot_trajectory::RobotTrajectory& traj)
{
  if (traj.getWayPointCount() == 0) {
    RCLCPP_WARN(LOGGER, "Trajectory has 0 waypoints, cannot move robot");
    return;
  }

  const moveit::core::RobotState& first_wp = traj.getWayPoint(0);
  std::vector<double> joint_positions;
  first_wp.copyJointGroupPositions(dual_arm_group_name, joint_positions);

  move_group_.setJointValueTarget(joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to first waypoint");
    return;
  }

  visual_tools_.prompt("Press Next to move robot to first waypoint...");
  success = (move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(LOGGER, "Failed to execute plan to first waypoint");
    return;
  }

  RCLCPP_INFO(LOGGER, "Robot moved to first waypoint successfully");
}

void TrajAdaptTestNode::evaluateClearanceForTrajectory(
    const planning_scene::PlanningSceneConstPtr& base_scene,
    const robot_trajectory::RobotTrajectory& traj,
    const std::string& object_id)
{
  if (!base_scene) {
    RCLCPP_WARN(rclcpp::get_logger("Clearance"), "Base PlanningScene is null");
    return;
  }
  if (traj.getWayPointCount() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("Clearance"), "Trajectory has 0 waypoints");
    return;
  }

  // --- Build evaluation scene with cable attached ---
  planning_scene::PlanningScenePtr eval_scene = base_scene->diff();
  const auto& rs = base_scene->getCurrentState();
  Eigen::Isometry3d leader_hand_transform = rs.getGlobalLinkTransform(lead_hand_frame) * lead_hand_to_tcp_transform_;
  Eigen::Isometry3d follower_hand_transform = rs.getGlobalLinkTransform(follow_hand_frame) * follow_hand_to_tcp_transform_;
  Eigen::Vector3d cable_vector_in_world = (leader_hand_transform.translation() - follower_hand_transform.translation()).normalized();

  attachCollisionCable(
      eval_scene, 
      object_id,
      /*length=*/0.1,
      /*radius=*/0.01,
      cable_vector_in_world,
      /*attach_link=*/follow_hand_frame,
      {follow_hand_frame, "left_panda_leftfinger", "left_panda_rightfinger",
       lead_hand_frame, "right_panda_leftfinger", "right_panda_rightfinger"},
      /*attach_to_robot=*/true);

  // --- ACM variants (same as before) ---
  collision_detection::AllowedCollisionMatrix acm_base = eval_scene->getAllowedCollisionMatrix();

  collision_detection::AllowedCollisionMatrix acm_robot_only = acm_base;
  acm_robot_only.setDefaultEntry(object_id, true);
  acm_robot_only.setEntry(object_id, true);

  collision_detection::AllowedCollisionMatrix acm_with_object = acm_base;
  acm_with_object.removeEntry(object_id);

  // --- Compute clearances ---
  double clearance_robot_only  = computeMinClearance(eval_scene, traj, acm_robot_only);
  double clearance_with_object = computeMinClearance(eval_scene, traj, acm_with_object);

  traj_clearance_["clearance_robot_only"]  = clearance_robot_only;
  traj_clearance_["clearance_with_object"] = clearance_with_object;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("Clearance"),
       "clearance(robot only)=" << clearance_robot_only
      << ", clearance(with object)=" << clearance_with_object);
}


// -------------------- Main API: read → stretch → dump --------------------

void TrajAdaptTestNode::updatePlanningScene()
{
  if (!current_joint_state_)
  {
    RCLCPP_WARN(LOGGER, "Joint state not yet received, cannot update planning scene.");
    return;
  }

  // Retrieve the current robot state from MoveGroup
  moveit::core::RobotStatePtr robot_state = move_group_.getCurrentState();

  // Update robot state with the latest joint positions
  const std::vector<std::string>& joint_names = current_joint_state_->name;
  const std::vector<double>& joint_positions = current_joint_state_->position;

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    robot_state->setJointPositions(joint_names[i], &joint_positions[i]);
  }

  // Apply the updated state to the planning scene
  move_group_.setStartState(*robot_state);
  RCLCPP_INFO(LOGGER, "Planning scene successfully updated.");
}

void TrajAdaptTestNode::runFromFiles(
    const std::string& leader_joint_txt,
    const std::string& follower_joint_txt,
    const std::string& leader_tcp_txt,
    const std::string& follower_tcp_txt,
    const std::vector<double>& stretch_distance,
    double ik_timeout,
    const std::filesystem::path& dump_dir)
{
  // Read inputs (tcp files are optional for the stretcher; we read them to validate/compare if you want)
  const JointTxtData lead_j = readJointTxt(leader_joint_txt, 7);
  const JointTxtData fol_j  = readJointTxt(follower_joint_txt, 7);
  (void)readTcpTxt(leader_tcp_txt);
  (void)readTcpTxt(follower_tcp_txt);

  // Build a dual-arm trajectory so every waypoint contains both arms
  robot_trajectory::RobotTrajectory dual_traj = buildDualRobotTrajectory(lead_j, fol_j);

  // Stretch the leader trajectory by the specified distance
  stretchRobotTrajectoryInPlace(dual_traj, stretch_distance[0], ik_timeout);

  // Move robot to first waypoint before clearance evaluation
  moveRobotToFirstWaypoint(dual_traj);
  updatePlanningScene();

  visual_tools_.prompt("Press Next to evaluate clearance after stretching...");

  publishDualTraj("stretched_clip", dual_traj, /*stage_id=*/0, /*id=*/0);

  // evaluate clearance after stretching
  planning_scene_monitor::PlanningSceneMonitor psm(node_, "robot_description");
  psm.startSceneMonitor();
  // psm.startWorldGeometryMonitor();
  psm.startStateMonitor();

  auto base_scene = psm.getPlanningScene();   // or however you get it
  nlohmann::json stretched_traj_clearance = nlohmann::json::object();

  planning_scene::PlanningScenePtr base_mut = base_scene->diff();
  base_mut->getCurrentStateNonConst() = dual_traj.getWayPoint(0);
  base_mut->getCurrentStateNonConst().update();

  evaluateClearanceForTrajectory(
      base_mut,
      dual_traj,
      "grasped_cable"
  );

  // Dump out: leader and follower groups separately, using your existing helper
  moveit::task_constructor::stages::dumpTrajectoryTXTIndexed(
      dual_traj,
      "leader_stretched",
      lead_arm_group_name,
      dump_dir,
      lead_flange_to_tcp_transform_,   // or lead_hand_to_tcp_transform_, depending on what your dump expects
      lead_base_frame);

  moveit::task_constructor::stages::dumpTrajectoryTXTIndexed(
      dual_traj,
      "follower_stretched",
      follow_arm_group_name,
      dump_dir,
      follow_flange_to_tcp_transform_,
      follow_base_frame);

  RCLCPP_INFO(LOGGER, "Done. Stretched trajectories dumped to: %s", dump_dir.string().c_str());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto traj_adapt_node = std::make_shared<TrajAdaptTestNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &traj_adapt_node]() {
    executor.add_node(traj_adapt_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(traj_adapt_node->getNodeBaseInterface());
  });

  std::string traj_path;
  if (!traj_adapt_node->getROSParam("traj_path", traj_path)) {
    RCLCPP_ERROR(LOGGER, "Parameter 'traj_path' not set");
    return 1;
  }

  int clip_id, stage_id, subtraj_id;
  traj_adapt_node->getROSParam("clip_id", clip_id);
  traj_adapt_node->getROSParam("stage_id", stage_id);
  traj_adapt_node->getROSParam("subtraj_id", subtraj_id);

  std::string leader_joint_txt = traj_path + "/leader/real_world_task_" + std::to_string(clip_id) + "_stage_" + std::to_string(stage_id) + "_traj_" + std::to_string(subtraj_id) + ".txt";
  std::string follower_joint_txt = traj_path + "/follower/real_world_task_" + std::to_string(clip_id) + "_stage_" + std::to_string(stage_id) + "_traj_" + std::to_string(subtraj_id) + ".txt";
  std::string leader_tcp_txt = traj_path + "/leader/clip" + std::to_string(clip_id) + "_stage_" + std::to_string(stage_id) + "_tcp_trajectory_" + std::to_string(subtraj_id) + ".txt";
  std::string follower_tcp_txt = traj_path + "/follower/clip" + std::to_string(clip_id) + "_stage_" + std::to_string(stage_id) + "_tcp_trajectory_" + std::to_string(subtraj_id) + ".txt";
  
  std::string dump_dir_str = traj_path + "_stretched";
  std::filesystem::create_directory(dump_dir_str);
  std::filesystem::path dump_dir(dump_dir_str);

  traj_adapt_node->getVisualTools().prompt("[Planning] Press 'next' in the RvizVisualToolsGui window to continue adaption...");

  double ik_timeout = 0.5; // seconds
  std::vector<double> stretch_distance_vector;
  traj_adapt_node->getROSParam("stretch_distance_list", stretch_distance_vector);

  traj_adapt_node->runFromFiles(leader_joint_txt, follower_joint_txt, leader_tcp_txt, follower_tcp_txt,
                                 stretch_distance_vector, ik_timeout, dump_dir);

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
