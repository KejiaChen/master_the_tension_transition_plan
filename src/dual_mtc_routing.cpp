#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <moveit_task_constructor_msgs/srv/get_clip_names.hpp>
#include <moveit/task_constructor/storage.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <cstdlib>
#include <tuple>
#include <vector>

#include <master_the_tension_transition_plan/dual_mtc_routing.h>
#include <moveit/task_constructor/stages/noop.h>
#include <moveit/task_constructor/cost_terms.h>

using boost::asio::ip::udp;
using json = nlohmann::json;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_node");

namespace {

std::string defaultWorkspaceDir() {
  if (const char* workspace_dir = std::getenv("MTC_WORKSPACE_DIR"); workspace_dir && workspace_dir[0] != '\0') {
    return workspace_dir;
  }
  if (const char* home_dir = std::getenv("HOME"); home_dir && home_dir[0] != '\0') {
    return std::string(home_dir) + "/ws_humble";
  }
  return "/tmp/ws_humble";
}

std::string workspacePath(const std::string& suffix) {
  return (std::filesystem::path(defaultWorkspaceDir()) / suffix).string();
}

std::unordered_map<std::string, std::set<int>> parseStageTargets(const std::vector<std::string>& targets) {
  std::unordered_map<std::string, std::set<int>> stage_to_indices;

  for (const auto& entry : targets) {
    const std::string pat = "_subtraj_";
    auto pos = entry.rfind(pat);

    if (pos == std::string::npos) {
      stage_to_indices[entry] = {};
      continue;
    }

    std::string stage_name = entry.substr(0, pos);
    std::string idx_str = entry.substr(pos + pat.size());

    try {
      int idx = std::stoi(idx_str);
      stage_to_indices[stage_name].insert(idx);
    } catch (const std::exception& e) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("RobotTrajectory"),
                         "evaluateClearance: cannot parse target entry '" << entry << "': " << e.what());
    }
  }

  return stage_to_indices;
}

}  // namespace

double desired_ee_distance = 0.12;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) },
    move_group_(node_, "dual_arm"),  // for dual arm or single arm
    visual_tools_(node_, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_.getRobotModel(), true),
    tf_buffer_(node_->get_clock()), // Initialize TF Buffer with node clock
    tf_listener_(tf_buffer_),       // Initialize TF Listener with TF Buffer
    lead_flange_to_tcp_transform_(Eigen::Isometry3d::Identity()),
    follow_flange_to_tcp_transform_(Eigen::Isometry3d::Identity())
{
  visual_tools_.loadRemoteControl();

  // Subtrajectory publisher
  subtrajectory_publisher_ = node_->create_publisher<moveit_task_constructor_msgs::msg::SubTrajectory>(
		    "/mtc_sub_trajectory", rclcpp::QoS(1).transient_local());

  // Subscription to the joint states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MTCTaskNode::jointStateCallback, this, std::placeholders::_1));

  // Wait until joint states are received
  RCLCPP_INFO(LOGGER, "Waiting for joint states...");
  while (!current_joint_state_)
  {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(LOGGER, "Joint states received.");

  // scene configuration
  insertion_offset_magnitude_ = 0.03; 
  grasp_follower_offset_magnitude_ = 0.04;
  grasp_leader_offset_magnitude_ = desired_ee_distance + grasp_follower_offset_magnitude_;
  hold_y_offset_ = 0.04;
  hold_z_offset_ = 0.01;
  // std::vector<double> leader_pre_insert_offset_ = {-(clip_size[0]/2+hold_x_offset), -clip_size[1]/2, clip_size[2]/2};
  // std::vector<double> follower_pre_insert_offset_ = {clip_size[0]/2+hold_x_offset, -clip_size[1]/2, clip_size[2]/2};
  // update with default clip size
  updateClipOffsets();

  // initialize udp sync with real-world
  udp_thread_lead_sync_ = std::thread(&MTCTaskNode::udpReceiverSync, // member function pointer
                                      this,                          // object pointer (this class instance)
                                      "10.157.175.222",                 // host
                                      6305,                          // port
                                      std::ref(lead_joint_positions_),              // reference to class member
                                      std::ref(lead_joint_positions_mutex_),        // mutex
                                      std::ref(lead_joint_positions_condition_variable_), // condition variable
                                      std::ref(lead_ee_pose_),                      // EE pose vector
                                      std::ref(lead_ee_pose_mutex_)                 // EE pose mutex
                                      );

  udp_thread_follow_sync_ = std::thread(&MTCTaskNode::udpReceiverSync, // member function pointer
                                        this,                          // object pointer (this class instance)
                                        "10.157.175.222",
                                        6306,                          // port
                                        std::ref(follow_joint_positions_),              // EE pose mutex
                                        std::ref(follow_joint_positions_mutex_),        // mutex
                                        std::ref(follow_joint_positions_condition_variable_), // condition variable
                                        std::ref(follow_ee_pose_),                      // EE pose vector
                                        std::ref(follow_ee_pose_mutex_)                 // EE pose mutex
                                        );

  
  bool attach_pull = node_->get_parameter("attach_pull_cable").as_bool();
  bool attach_transport = node_->get_parameter("attach_transport_cable").as_bool();
  clearance_results_["attach_pull_cable"] = attach_pull;
  clearance_results_["attach_transport_cable"] = attach_transport;

  initializeTransforms(/*default_franka_flange_to_tcp_z*/ 0.1034,
                        /*sensone_height*/ 0.036,
                        /*extend_finger_length*/ 0.01);

  // Initialize Planning Groups
  initializeGroups();
}

// Destructor
MTCTaskNode::~MTCTaskNode()
{
  // stop the UDP receiver thread
  sync_udp_running_ = false; // Signal the UDP threads to stop
  lead_joint_positions_condition_variable_.notify_all();
  if (udp_thread_lead_sync_.joinable()) {
    udp_thread_lead_sync_.join();
  }

  follow_joint_positions_condition_variable_.notify_all();
  if (udp_thread_follow_sync_.joinable()) {
    udp_thread_follow_sync_.join();
  }
}

void MTCTaskNode::initializeGroups()
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

void MTCTaskNode::initializePlanners()
{ 
    std::string chomp_pipeline_name = "chomp";
    // Lead arm planners
    lead_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    // lead_sampling_planner->setPlannerId("RRTstarkConfigDefault");
    lead_sampling_planner->setMaxVelocityScalingFactor(0.05);
    lead_sampling_planner->setMaxAccelerationScalingFactor(0.05);

    lead_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    lead_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    lead_cartesian_planner->setMaxVelocityScalingFactor(0.05);
    lead_cartesian_planner->setMaxAccelerationScalingFactor(0.05);
    lead_cartesian_planner->setStepSize(0.01);

    lead_chomp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, chomp_pipeline_name);
    lead_chomp_planner->setMaxVelocityScalingFactor(0.05);
    lead_chomp_planner->setMaxAccelerationScalingFactor(0.05);

    // Follow arm planners
    follow_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    // follow_sampling_planner->setPlannerId("BiESTkConfigDefault");
    follow_sampling_planner->setMaxVelocityScalingFactor(0.05);
    follow_sampling_planner->setMaxAccelerationScalingFactor(0.05);

    follow_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    follow_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    follow_cartesian_planner->setMaxVelocityScalingFactor(0.05);
    follow_cartesian_planner->setMaxAccelerationScalingFactor(0.05);
    follow_cartesian_planner->setStepSize(0.01);

    follow_chomp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, chomp_pipeline_name);
    follow_chomp_planner->setMaxVelocityScalingFactor(0.05);
    follow_chomp_planner->setMaxAccelerationScalingFactor(0.05);
}

void MTCTaskNode::initializeTransforms(double default_franka_flange_to_tcp_z,
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

 void MTCTaskNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = msg; // Cache the latest joint state
}

void MTCTaskNode::updatePlanningScene()
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

rclcpp::Node::SharedPtr MTCTaskNode::getNode()
{
  return node_;
}

moveit_visual_tools::MoveItVisualTools& MTCTaskNode::getVisualTools()
{
  return visual_tools_;
}

void MTCTaskNode::udpReceiverSync(const std::string& host, int port,
  // std::array<double, 6>& joint_positions,
  std::vector<double>& joint_positions,
  std::mutex& joint_positions_mutex,
  std::condition_variable& joint_positions_condition_variable,
  std::vector<double>& ee_pose,
  std::mutex& ee_pose_mutex
  )
{
  try {
    boost::asio::io_context io_context;

    // Bind to all interfaces to avoid issues if the IP is wrong
    udp::socket socket(io_context, udp::endpoint(boost::asio::ip::udp::v4(), port));

    std::array<char, 4096> recv_buf;

    while (sync_udp_running_) {
      udp::endpoint sender_endpoint;
      size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
      std::string data(recv_buf.data(), len);
    
      std::cout << "[UDP raw data] " << data << std::endl;
    
      // Safely parse JSON
      json received_json;
      try {
        received_json = json::parse(data);
      } catch (const json::parse_error& e) {
        std::cerr << "[UDP] JSON parse error: " << e.what() << "\n[UDP] Dropping message: " << data << std::endl;
        continue;  // Skip garbage
      }
    
      // Verify 'command' or 'magic' key to ensure it's from your system
      if (!received_json.contains("command") || received_json["command"] != "synchronize") {
        std::cerr << "[UDP] Invalid or missing 'command'. Skipping message." << std::endl;
        continue;
      }
    
      // Check and validate joint data
      if (!received_json.contains("q") || !received_json["q"].is_array()) {
        std::cerr << "[UDP] Missing or invalid 'q' array." << std::endl;
        continue;
      }
    
      std::vector<double> current_q = received_json["q"].get<std::vector<double>>();
      if (current_q.size() != 7) {
        std::cerr << "[UDP] Incorrect joint vector size: " << current_q.size() << std::endl;
        continue;
      }
    
      {
        std::lock_guard<std::mutex> lock(joint_positions_mutex);
        joint_positions = current_q;
    
        if (port == 6305) lead_joint_data_received_ = true;
        if (port == 6306) follow_joint_data_received_ = true;
      }
    
      joint_positions_condition_variable.notify_one();
    
      std::cout << "[UDP] Accepted joint data: ";
      for (double q : current_q) std::cout << q << " ";
      std::cout << std::endl;
    }    

  } catch (const std::exception& e) {
    std::cerr << "UDP Receiver Error: " << e.what() << std::endl;
  }
}

bool MTCTaskNode::doSyncTask(std::string arm_group_name, 
                              std::string hand_group_name, 
                              std::string hand_frame,
                              std::mutex& joint_positions_mutex,
                              std::vector<double>& joint_positions,
                              std::vector<std::string>& joint_names,
                              std::condition_variable& joint_positions_condition_variable,
                              std::atomic<bool>& joint_data_received_flag,
                              std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner
  )
{
  RCLCPP_INFO(LOGGER, "Waiting for joint position via UDP...");

  joint_data_received_flag = false;

  {
  std::unique_lock<std::mutex> lock(joint_positions_mutex);
  joint_positions_condition_variable.wait(lock, [&] {
  return joint_data_received_flag.load() && joint_positions.size() == 7;
  });
  }

  RCLCPP_INFO(LOGGER, "Joint position received: [%f, %f, %f, %f, %f, %f, %f]",
  joint_positions[0], joint_positions[1], joint_positions[2],
  joint_positions[3], joint_positions[4], joint_positions[5],
  joint_positions[6]);

  joint_data_received_flag = false;  // Reset

  RCLCPP_INFO(LOGGER, "Joint position received: [%f, %f, %f, %f, %f, %f, %f]", joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4], joint_positions[5], joint_positions[6]);

  mtc::Task sync_task = createGoalJointTask(arm_group_name, hand_group_name, hand_frame,
                    joint_positions_mutex, 
                    joint_positions, 
                    joint_names,
                    sampling_planner
                    );
  try
  {
  sync_task.init();
  }
  catch (mtc::InitStageException& e)
  {
  RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task initialization failed: " << e.what());
  return false;
  }

  if (!sync_task.plan(5))
  {
  RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task planning failed");
  return false;
  }

  // Do not publish the solution for synchronization task
  // sync_task.introspection().publishSolution(*sync_task.solutions().front());
  auto sync_result = sync_task.execute(*sync_task.solutions().front());
  if (sync_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
  RCLCPP_ERROR_STREAM(LOGGER, "Arm sync task execution failed");
  return false;
  }
  RCLCPP_INFO(LOGGER, "Arm sync task executed successfully");

  return true;
}

mtc::Task MTCTaskNode::createGoalJointTask(std::string arm_group_name, 
                                          std::string hand_group_name, 
                                          std::string hand_frame,
                                          std::mutex& joint_positions_mutex,
                                          std::vector<double>& joint_positions,
                                          std::vector<std::string>& joint_names,
                                          std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner
                                          )
{
  mtc::Task task;
  task.stages()->setName("synchronization arm task");
  task.loadRobotModel(node_);

  std::vector<double> delta = {0, 0, 0};
  std::vector<double> orients = {0, 0, 0, 1};

  // task.setProperty("left_group", arm_group_name);
  // task.setProperty("left_eef", hand_group_name);
  // task.setProperty("left_ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop


  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // move to synchronization joint position
  {
    // set Goal from joint positions
    std::map<std::string, double> joint_goal;
    {
    if (joint_positions.size() != joint_names.size()) {
    std::cerr << "Error: joint_positions size (" << joint_positions.size() 
    << ") does not match joint_names size (" << joint_names.size() << ")" << std::endl;
    return task;
    }

    std::lock_guard<std::mutex> joint_lock(joint_positions_mutex);

    for (size_t i = 0; i < joint_names.size(); i++)
    {
    joint_goal[joint_names[i]] = joint_positions[i];
    }

    std::cout << "printing joint goal" << std::endl;
    for (auto const& [key, val] : joint_goal)
    {
    std::cout << key << ": " << val << std::endl;
    }

    }

    // auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", joint_interpolation_planner);
    auto stage_move_to_joint = std::make_unique<mtc::stages::MoveTo>("move to synchronization position", sampling_planner);
    stage_move_to_joint->setGroup(arm_group_name);
    stage_move_to_joint->setGoal(joint_goal);
    task.add(std::move(stage_move_to_joint));
  }

  return task;
}


void MTCTaskNode::syncwithRealWorld()
{
  initializeGroups();

  initializePlanners();

  RCLCPP_INFO(LOGGER, "Synchronizing the lead arm to its real-world position...");
  doSyncTask(lead_arm_group_name, lead_hand_group_name, lead_hand_frame,
             lead_joint_positions_mutex_, 
             lead_joint_positions_, lead_franka_joint_names_,
             lead_joint_positions_condition_variable_,
             lead_joint_data_received_,
             lead_sampling_planner
             );

  RCLCPP_INFO(LOGGER, "Synchronizing the follow arm to its real-world position...");
  doSyncTask(follow_arm_group_name, follow_hand_group_name, follow_hand_frame,
             follow_joint_positions_mutex_,
             follow_joint_positions_, follow_franka_joint_names_,
             follow_joint_positions_condition_variable_,
              follow_joint_data_received_,
              follow_sampling_planner
             );

  RCLCPP_INFO(LOGGER, "Synchronization with real-world completed.");
}


void MTCTaskNode::publishSolutionSubTraj(std::string goal_clip_name, const moveit_task_constructor_msgs::msg::Solution& msg) {
	int index = 0;
  // int task_id = std::stoi(msg.task_id);
  int task_id = 0;
  // try {
  //   if (!msg.task_id.empty()) {
  //       task_id = std::stoi(msg.task_id);
  //   } else {
  //       RCLCPP_WARN(LOGGER, "task_id is empty.");
  //   }
  // }catch (const std::invalid_argument& e) {
  //     RCLCPP_ERROR(LOGGER, "Invalid task_id: %s", msg.task_id.c_str());
  //     return;  // Handle the error appropriately
  // } catch (const std::out_of_range& e) {
  //     RCLCPP_ERROR(LOGGER, "task_id out of range: %s", msg.task_id.c_str());
  //     return;  // Handle the error appropriately
  // }

  for (const moveit_task_constructor_msgs::msg::SubTrajectory& sub_trajectory : msg.sub_trajectory) {
    if (sub_trajectory.trajectory.joint_trajectory.points.empty())
      continue;

    // visualize trajectories
    moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
    robot_trajectory_msg.joint_trajectory = sub_trajectory.trajectory.joint_trajectory;

    visual_tools_.publishTrajectoryLine(robot_trajectory_msg, move_group_.getCurrentState()->getJointModelGroup(lead_arm_group_name),
                                        lead_flange_to_tcp_transform_, goal_clip_name, sub_trajectory.info.stage_id, index,
                                        workspacePath("trajectories_leader"), rviz_visual_tools::ORANGE, lead_base_frame);
    visual_tools_.publishTrajectoryLine(robot_trajectory_msg, move_group_.getCurrentState()->getJointModelGroup(follow_arm_group_name),
                                        follow_flange_to_tcp_transform_, goal_clip_name, sub_trajectory.info.stage_id, index,
                                        workspacePath("trajectories_follower"), rviz_visual_tools::BLUE, follow_base_frame);
    // visual_tools_.publishTrajectoryLine(robot_trajectory_msg, move_group_.getCurrentState()->getLinkModel(lead_hand_frame));
    visual_tools_.trigger();

    // renumber trajectory id
    auto new_sub_trajectory = sub_trajectory;
    new_sub_trajectory.info.id = index;

    // publish trajectories
    subtrajectory_publisher_->publish(new_sub_trajectory);
    RCLCPP_INFO(LOGGER, "Sub-trajectory includes the following joints:");
    for (const auto& joint_name : sub_trajectory.trajectory.joint_trajectory.joint_names) {
        RCLCPP_INFO(LOGGER, "  %s", joint_name.c_str());
    }
    RCLCPP_INFO_STREAM(LOGGER, "Published subtrajectory id " << new_sub_trajectory.info.id 
                              << " for stage " << new_sub_trajectory.info.stage_id
                              << " with " << new_sub_trajectory.trajectory.joint_trajectory.points.size()
                              << " waypoints");
    visual_tools_.prompt("[Publishing] Press 'next' to publishing the next subtrajectory");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    index++;
  }
  return;
	
}

geometry_msgs::msg::PoseStamped MTCTaskNode::getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame)
{   
    std::string frame_id = pose.header.frame_id;
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            target_frame,   // Target frame
            frame_id,  // Source frame
            rclcpp::Time(0),  // Get the latest transform
            rclcpp::Duration::from_seconds(1.0)// Timeout
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "Could not transform %s frame to %s frame: %s", frame_id.c_str(), target_frame.c_str(), ex.what());
        return pose;
    }

    geometry_msgs::msg::PoseStamped pose_world;
    tf2::doTransform(pose, pose_world, transform);
    return pose_world;
}

  // Helper: look up T^world_clip as Eigen Isometry
  Eigen::Isometry3d MTCTaskNode::getTransformIsometry(const std::string& source_frame, const std::string& target_frame)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            target_frame,   // Target frame
            source_frame,  // Source frame
            rclcpp::Time(0),  // Get the latest transform
            rclcpp::Duration::from_seconds(1.0)// Timeout
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "Could not transform %s frame to %s frame: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return Eigen::Isometry3d::Identity(); // Return identity on failure
    }

    // Convert to Eigen::Isometry3d
    Eigen::Isometry3d T = tf2::transformToEigen(transform.transform);

    // Guard against tiny numerical drift (keeps rotation orthonormal)
    Eigen::Matrix3d R = T.linear();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R_orth = svd.matrixU() * svd.matrixV().transpose();
    if (R_orth.determinant() < 0.0) {  // fix potential reflection
      Eigen::Matrix3d U = svd.matrixU();
      U.col(2) *= -1.0;
      R_orth = U * svd.matrixV().transpose();
    }
    T.linear() = R_orth;  // reassign the cleaned rotation

    return T;
  };

moveit_msgs::msg::Constraints MTCTaskNode::createBoxConstraints(const std::string& link_name, geometry_msgs::msg::PoseStamped& goal_pose, double x_offset, double y_offset, double z_offset)
{
  // Assume current_pose and goal_pose are of type geometry_msgs::msg::PoseStamped
  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose(link_name);
  geometry_msgs::msg::PoseStamped current_pose_transformed = getPoseTransform(current_pose, "world");
  geometry_msgs::msg::PoseStamped goal_pose_transformed = getPoseTransform(goal_pose, "world");
  RCLCPP_INFO(LOGGER, "Goal pose transformed: x: %f, y: %f, z: %f", goal_pose_transformed.pose.position.x, goal_pose_transformed.pose.position.y, goal_pose_transformed.pose.position.z);

  // Compute the box center and dimensions
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = (current_pose_transformed.pose.position.x + goal_pose_transformed.pose.position.x) / 2.0;
  box_pose.position.y = (current_pose_transformed.pose.position.y + goal_pose_transformed.pose.position.y) / 2.0;
  box_pose.position.z = (current_pose_transformed.pose.position.z + goal_pose_transformed.pose.position.z) / 2.0;
  box_pose.orientation.w = 1.0; // Identity quaternion for box orientation

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
      fabs(goal_pose_transformed.pose.position.x - current_pose_transformed.pose.position.x)+x_offset,  // Length (x)
      fabs(goal_pose_transformed.pose.position.y - current_pose_transformed.pose.position.y)+y_offset,  // Width (y)
      fabs(goal_pose_transformed.pose.position.z - current_pose_transformed.pose.position.z)+z_offset   // Height (z)
  };

  // Create position constraint
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = "world"; // Replace with the appropriate reference frame
  box_constraint.link_name = link_name; // Replace with the relevant link name
  box_constraint.constraint_region.primitives.emplace_back(box);
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  // Visualize the box constraint
  Eigen::Vector3d box_point_1(
      box_pose.position.x - box.dimensions[0] / 2.0,
      box_pose.position.y - box.dimensions[1] / 2.0,
      box_pose.position.z - box.dimensions[2] / 2.0
  );
  Eigen::Vector3d box_point_2(
      box_pose.position.x + box.dimensions[0] / 2.0,
      box_pose.position.y + box.dimensions[1] / 2.0,
      box_pose.position.z + box.dimensions[2] / 2.0
  );
  visual_tools_.publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  visual_tools_.trigger();

  // Wrap in a generic Constraints message
  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  return box_constraints;
}


// ===== Main function =====
std::tuple<int, geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> MTCTaskNode::assignClipGoalsAlongConnection(const std::string& clip_frame,
                                                                                                                      const std::string& next_clip_frame,
                                                                                                                      const std::vector<double>& leader_grasp_offset,
                                                                                                                      const std::vector<double>& follower_grasp_offset,
                                                                                                                      bool tilt_follower,
                                                                                                                      double follower_tilt_rad,
                                                                                                                      Eigen::Quaterniond& clip2ee_quat)
{
  // 1) Look up both clip poses in world
  const auto T_w_c = getTransformIsometry(clip_frame, "world");
  const auto T_w_n = getTransformIsometry(next_clip_frame, "world");

  const Eigen::Vector3d p_c = T_w_c.translation();
  const Eigen::Vector3d p_n = T_w_n.translation();

  // 2) Connection vector current->next in world
  Eigen::Vector3d v_conn_w = (p_n - p_c);
  if (v_conn_w.norm() < 1e-9) {
    throw std::runtime_error("assignClipGoalsAlongConnection: clips are coincident.");
  }
  v_conn_w.normalize();

  // 3) Decide forward side in the CLIP frame (+Y or -Y)
  int clip_sign = signAlongClipY(T_w_c, v_conn_w); // +1 => use +Y_clip, else -Y_clip

  // 4) Base orientation: Q2 for +Y, Q1 for -Y
  static const tf2::Quaternion Q1(0.7071068, -0.7071068, 0.0, 0.0); // 180° about X then -90° about Z
  static const tf2::Quaternion Q2(0.7071068,  0.7071068, 0.0, 0.0); // 180° about X then +90° about Z
  const tf2::Quaternion q_tf = (clip_sign > 0) ? Q2 : Q1;
  // const Eigen::Quaterniond q_base(q_tf.getW(), q_tf.getX(), q_tf.getY(), q_tf.getZ());
  clip2ee_quat = Eigen::Quaterniond(q_tf.getW(), q_tf.getX(), q_tf.getY(), q_tf.getZ());

  // 5) Build leader/follower goals in the CLIP frame
  geometry_msgs::msg::PoseStamped leader, follower;
  leader.header.frame_id   = clip_frame;
  follower.header.frame_id = clip_frame;

  // Positions: ±Y in CLIP frame
  leader.pose.position.x   = leader_grasp_offset[0]; // leader offset along clip X
  leader.pose.position.y   = clip_sign * leader_grasp_offset[1];   // leader ahead along connection
  leader.pose.position.z   = leader_grasp_offset[2]; // leader offset along clip Z

  follower.pose.position.x = follower_grasp_offset[0]; // follower offset along clip X
  follower.pose.position.y = clip_sign * follower_grasp_offset[1];   // opposite side
  follower.pose.position.z = follower_grasp_offset[2]; // follower offset along clip Z

  // Orientations
  assignQuat(leader.pose,   clip2ee_quat);
  assignQuat(follower.pose, clip2ee_quat);

  // 6) Optional follower tilt about X (applied after base orientation)
  if (tilt_follower && std::abs(follower_tilt_rad) > 1e-6) {
    // Build a rotation about local Y (intrinsic): q_delta_y
    const Eigen::AngleAxisd aa_local_y(follower_tilt_rad, Eigen::Vector3d::UnitY());
    const Eigen::Quaterniond q_delta_y(aa_local_y);

    // Compose: local-Y tilt AFTER base orientation => right-multiply
    Eigen::Quaterniond qf(follower.pose.orientation.w,
                          follower.pose.orientation.x,
                          follower.pose.orientation.y,
                          follower.pose.orientation.z);
    Eigen::Quaterniond q_new = qf * q_delta_y;
    q_new.normalize();

    follower.pose.orientation.w = q_new.w();
    follower.pose.orientation.x = q_new.x();
    follower.pose.orientation.y = q_new.y();
    follower.pose.orientation.z = q_new.z();
  }

  return std::make_tuple(clip_sign, leader, follower);
}



std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> MTCTaskNode::assignClipGoal(const std::string& goal_frame_name, 
          const std::vector<double>& goal_vector_1, const std::vector<double>& goal_vector_2)
{
  /*For each clip, leader should always be on the right side of the clip. This gives to two possibilities of leader's sides in the clip frame.*/
  geometry_msgs::msg::PoseStamped leader_target_pose;
  geometry_msgs::msg::PoseStamped follower_target_pose;

  geometry_msgs::msg::PoseStamped target_pose_1 = createClipGoal(goal_frame_name, goal_vector_1);
  geometry_msgs::msg::PoseStamped target_pose_1_transformed = getPoseTransform(target_pose_1, "world");
  
  geometry_msgs::msg::PoseStamped target_pose_2 = createClipGoal(goal_frame_name, goal_vector_2);
  geometry_msgs::msg::PoseStamped target_pose_2_transformed = getPoseTransform(target_pose_2, "world");

  // Assign the goal pose based on y position
  Eigen::AngleAxisd follow_rotation(0, Eigen::Vector3d::UnitX());
  if (target_pose_1_transformed.pose.position.y > target_pose_2_transformed.pose.position.y){
    leader_target_pose = target_pose_1;
    follower_target_pose = target_pose_2;

    // Rotate follow_target_pose around the x-axis by 45 degrees
    // follow_rotation = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitX());
  } else {
    leader_target_pose = target_pose_2;
    follower_target_pose = target_pose_1;

    // Rotate follow_target_pose around the x-axis by 45 degrees
    // follow_rotation = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
  }

  // leader_target_pose = target_pose_1;
  // follower_target_pose = target_pose_2;

  // // Rotate follow_target_pose around the x-axis by 45 degrees
  // Eigen::AngleAxisd follow_rotation(-M_PI / 4, Eigen::Vector3d::UnitX());

  Eigen::Quaterniond current_orientation(follower_target_pose.pose.orientation.w,
                                        follower_target_pose.pose.orientation.x,
                                        follower_target_pose.pose.orientation.y,
                                        follower_target_pose.pose.orientation.z);

  // Apply the rotation
  Eigen::Quaterniond updated_orientation = follow_rotation * current_orientation;
  // Update the pose's orientation
  follower_target_pose.pose.orientation.w = updated_orientation.w();
  follower_target_pose.pose.orientation.x = updated_orientation.x();
  follower_target_pose.pose.orientation.y = updated_orientation.y();
  follower_target_pose.pose.orientation.z = updated_orientation.z();

  return std::make_pair(leader_target_pose, follower_target_pose);
}

geometry_msgs::msg::PoseStamped MTCTaskNode::createClipGoal(const std::string& goal_frame, const std::vector<double>& goal_translation_vector)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.pose.position.x = goal_translation_vector[0];
  goal_pose.pose.position.y = goal_translation_vector[1];
  goal_pose.pose.position.z = goal_translation_vector[2];

  goal_pose.header.frame_id = goal_frame;

  // Select orientaiton based on distance to current pose
  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose("right_panda_hand");
  // current pose in the goal frame
  geometry_msgs::msg::PoseStamped current_pose_transformed = getPoseTransform(current_pose, goal_frame);
  
  tf2::Quaternion current_orientation(
    current_pose_transformed.pose.orientation.x,
    current_pose_transformed.pose.orientation.y,
    current_pose_transformed.pose.orientation.z,
    current_pose_transformed.pose.orientation.w);
  
  // Get orientation in the goal frame
  tf2::Quaternion goal_orientation_1(0.7071068, -0.7071068, 0.0, 0.0);
  tf2::Quaternion goal_orientation_2(0.7071068, 0.7071068, 0.0, 0.0);

  tf2::Quaternion selected_orientation;
  if (select_orientation_){
    double angle_diff1 = current_orientation.angleShortestPath(goal_orientation_1);
    double angle_diff2 = current_orientation.angleShortestPath(goal_orientation_2);
    RCLCPP_INFO(LOGGER, "Angle diff 1: %f, Angle diff 2: %f", angle_diff1, angle_diff2);

    if (angle_diff1 < angle_diff2) {
        selected_orientation = goal_orientation_1;
    } else {
        selected_orientation = goal_orientation_2;
    }
  }else{
    selected_orientation = goal_orientation_1;
  }

  RCLCPP_INFO(LOGGER, "Selected orientation: x: %f, y: %f, z: %f, w: %f", selected_orientation.x(), selected_orientation.y(), selected_orientation.z(), selected_orientation.w());

  // Orientation from clip frame to robot EE frame
  goal_pose.pose.orientation.x = selected_orientation.x();
  goal_pose.pose.orientation.y = selected_orientation.y();
  goal_pose.pose.orientation.z = selected_orientation.z();
  goal_pose.pose.orientation.w = selected_orientation.w();

  return goal_pose;
}

void MTCTaskNode::evaluateClearance(
    const std::string& task_name,                    // NEW
    const std::string& clip_id,
    const std::string& object_id,
    const std::vector<std::string>& target_stages_and_indices)
{
  // Top-level: clip
  nlohmann::json& clip_node = clearance_results_[clip_id];

  // Under clip: task (e.g. "routing task", "post routing task")
  nlohmann::json& task_clearance = clip_node[task_name];

  if (task_.solutions().empty()) {
    RCLCPP_WARN(LOGGER, "No solutions; skip clearance eval");
    return;
  }

  const auto& root = *task_.solutions().front();
  const std::string root_stage_name = task_.stages()->name();

  // Total solution cost for THIS task and clip
  task_clearance["total_solution_cost"] = root.cost();

  // --------------------------------------------------------------------------
  // 1) Parse targets:
  //    "stage_subtraj_i" -> stage_to_indices[stage] = { i }
  //    "stage"           -> stage_to_indices[stage] = empty set (ALL subtrajs)
  // --------------------------------------------------------------------------
  auto stage_to_indices = parseStageTargets(target_stages_and_indices);

  if (stage_to_indices.empty()) {
    RCLCPP_WARN(LOGGER, "evaluateClearance: no valid targets parsed");
    return;
  }

  // --------------------------------------------------------------------------
  // 2) Recursively walk the solution tree
  //    - record stage_cost from *any* SolutionBase with a creator()
  //    - compute clearance for matching SubTrajectories
  // --------------------------------------------------------------------------
  std::unordered_map<std::string, int> stage_seen_count;

  std::function<void(const mtc::SolutionBase&)> recurse;
  recurse = [&](const mtc::SolutionBase& s)
  {
    // Stage-level cost (like MTC GUI)
    if (const auto* stage = s.creator()) {
      const std::string stage_name = stage->name();
      double stage_cost = s.cost();   // what GUI shows on that node

      // JSON node for this stage under THIS task
      // SKIP writing stage costs for the TASK ROOT
    if (stage_name == root_stage_name) {
        // we still use its cost for total_solution_cost above
        // but do NOT create a stage node with same name
        // simply return and continue recursion
    } else {
        nlohmann::json& stage_json = task_clearance[stage_name];

        if (!stage_json.contains("stage_cost")) {
          stage_json["stage_cost"] = stage_cost;
          RCLCPP_INFO_STREAM(LOGGER,
              "Stage '" << stage_name << "' solution cost (GUI-style): "
                        << stage_cost);
        }
      }
    }

    // Leaf: SubTrajectory -> clearance logic
    if (auto* sub = dynamic_cast<const mtc::SubTrajectory*>(&s)) {
      const auto* stage = sub->creator();
      if (!stage) return;

      const std::string stage_name = stage->name();

      auto it = stage_to_indices.find(stage_name);
      if (it != stage_to_indices.end()) {
        // Count subtrajectories for this stage
        int this_idx = stage_seen_count[stage_name]++;
        RCLCPP_INFO_STREAM(LOGGER,
           "Stage '" << stage_name << "' subtraj index " << this_idx);

        const auto& target_indices = it->second;
        bool evaluate_this_subtraj = false;

        if (target_indices.empty()) {
          // no specific index requested => ALL subtrajectories
          evaluate_this_subtraj = true;
        } else {
          evaluate_this_subtraj = target_indices.count(this_idx) > 0;
        }

        if (evaluate_this_subtraj) {
          RCLCPP_INFO_STREAM(LOGGER,
              "Evaluating clearance on stage '" << stage_name
              << "', subtraj " << this_idx);

          // JSON node for this stage under THIS task
          nlohmann::json& stage_json = task_clearance[stage_name];

          // -------- clearance computation (unchanged) --------
          const mtc::InterfaceState* start_interface_state = sub->start();
          if (!start_interface_state) {
            RCLCPP_WARN(LOGGER, "SubTrajectory has no start state");
            return;
          }

          // Base scene: no cable attached
          planning_scene::PlanningSceneConstPtr base_scene =
              start_interface_state->scene();
          const auto& rs = base_scene->getCurrentState();

          // Make a modifiable copy and attach cable for evaluation
          planning_scene::PlanningScenePtr eval_scene = base_scene->diff();
          Eigen::Isometry3d leader_hand_transform = rs.getGlobalLinkTransform(lead_hand_frame) * lead_hand_to_tcp_transform_;
          Eigen::Isometry3d follower_hand_transform = rs.getGlobalLinkTransform(follow_hand_frame) * follow_hand_to_tcp_transform_;
          Eigen::Vector3d cable_vector_in_world = (leader_hand_transform.translation() - follower_hand_transform.translation()).normalized();

          attachCollisionCable(
              eval_scene, object_id,
              0.1, 0.01, cable_vector_in_world, "left_panda_hand",
              {"left_panda_hand", "left_panda_leftfinger", "left_panda_rightfinger",
               "right_panda_hand", "right_panda_leftfinger", "right_panda_rightfinger"},
              true);

          // Get the trajectory for THIS subtrajectory
          auto traj_ptr = sub->trajectory();
          if (!traj_ptr) {
            RCLCPP_WARN(LOGGER, "SubTrajectory has no RobotTrajectory");
            return;
          }
          const robot_trajectory::RobotTrajectory& traj = *traj_ptr;

          // Base ACM from the eval scene
          collision_detection::AllowedCollisionMatrix acm_base = eval_scene->getAllowedCollisionMatrix();

          // A) Robot-only clearance: ignore cable collisions
          collision_detection::AllowedCollisionMatrix acm_robot_only = acm_base;
          acm_robot_only.setDefaultEntry(object_id, true);
          acm_robot_only.setEntry(object_id, true);

          // B) Robot + cable clearance: enforce cable collisions
          collision_detection::AllowedCollisionMatrix acm_with_object = acm_base;
          acm_with_object.removeEntry(object_id);

          double clearance_robot_only =
              computeMinClearance(eval_scene, traj, acm_robot_only);
          double clearance_with_object =
              computeMinClearance(eval_scene, traj, acm_with_object);

          RCLCPP_INFO_STREAM(LOGGER,
              "[" << stage_name << ", subtraj " << this_idx
              << "] clearance(robot only)=" << clearance_robot_only
              << ", clearance(with object)=" << clearance_with_object);

          // Store per-subtraj data under this stage
          stage_json["subtraj_" + std::to_string(this_idx)] = {
            {"clearance_robot_only",  clearance_robot_only},
            {"clearance_with_object", clearance_with_object}
          };
          // ------------------------------------------------------
        }
      }
    }

    // Container: SolutionSequence → recurse into children
    if (auto* seq = dynamic_cast<const mtc::SolutionSequence*>(&s)) {
      for (const mtc::SolutionBase* child : seq->solutions())
        recurse(*child);
    }
  };

  recurse(root);
}


void MTCTaskNode::doTask(std::string& start_clip_id, std::string& goal_clip_id, bool execute, bool plan_for_dual, bool split, bool cartesian_connect, bool approach, bool clip_added_from_blender,
                        std::function<mtc::Task(std::string&, std::string&, bool, bool, bool, bool, bool)> createTaskFn)
{
  task_ = createTaskFn(start_clip_id, goal_clip_id, plan_for_dual, split, cartesian_connect, approach, clip_added_from_blender);

  // publish solution for moveit servo
  bool publish_mtc_trajectory = !execute;

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Task planned successfully");

  if (task_.solutions().empty()) {
    RCLCPP_WARN(LOGGER, "No solutions, skip clearance eval");
    return;
  }

  evaluateClearance(
      task_.stages()->name(),
      goal_clip_id,
      "grasped_cable",
      {"move to align_subtraj_3"}
  );

  // Publish the solution
  // task_.introspection().publishSolution(*task_.solutions().front(), publish_mtc_trajectory);
  moveit_task_constructor_msgs::msg::Solution msg;
	task_.introspection().fillSolution(msg, *task_.solutions().front());
  publishSolutionSubTraj(goal_clip_id, msg);
  
  return;
}

mtc::Task MTCTaskNode::createTask(std::string& start_frame_name, std::string& goal_frame_name, 
                                  bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach, bool clip_added_from_blender)
{
  mtc::Task task;
  task.stages()->setName("routing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // Set task properties (only valid for single arm)
  // if (!if_use_dual){
  //   task.setProperty("group", lead_arm_group_name);
  //   task.setProperty("eef", lead_hand_group_name);
  //   task.setProperty("ik_frame", lead_hand_frame);
  // }
  
  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  const bool has_start_clip = !start_frame_name.empty();
  const bool use_cartesian_connect = if_cartesian_connect && has_start_clip;

  if (!if_use_dual) {
    RCLCPP_WARN(LOGGER, "createTask() currently supports only the dual-arm routing flow used by main()");
  }

  if (if_split_plan) {
    RCLCPP_WARN(LOGGER, "createTask() ignoring split planning because main() never uses that path");
  }

  // The first task has no previous clip, so reuse the goal clip size instead of querying with an empty frame id.
  std::vector<double> goal_clip_size = getClipSizeFromScene(goal_frame_name);
  std::vector<double> start_clip_size = has_start_clip ? getClipSizeFromScene(start_frame_name) : goal_clip_size;
  updateClipOffsets(start_clip_size, goal_clip_size, clip_added_from_blender);

  RCLCPP_INFO(LOGGER, "Inserting in clip frame : %s", goal_frame_name.c_str());
  auto [lead_target_pose, follow_target_pose] = assignClipGoal(goal_frame_name, leader_pre_insert_offset_, follower_pre_insert_offset_);

  if (if_cartesian_connect && !has_start_clip) {
    RCLCPP_WARN(LOGGER, "Cartesian connect requested without a start clip; falling back to direct align planning");
  }

  Eigen::Quaterniond quat_clip2ee = Eigen::Quaterniond::Identity();
  int clip_sign = 1;
  if (use_cartesian_connect) {
    RCLCPP_INFO(LOGGER, "Grasping in clip frame : %s", start_frame_name.c_str());
    std::tie(clip_sign, std::ignore, std::ignore) = assignClipGoalsAlongConnection(
        start_frame_name, goal_frame_name,
        leader_grasp_offset_magnitude_, follower_grasp_offset_magnitude_,
        true, M_PI / 4, quat_clip2ee);
  }

  mtc::Stage* pre_move_stage_ptr = nullptr;

  /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    pre_move_stage_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

  }
   
  // Set up planners
  initializePlanners();

       /****************************************************
    ---- *               Open Hand                      *
    ***************************************************/
   {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", follow_interpolation_planner);
    stage->setGroup(follow_hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  /****************************************************
	 *                                                  *
	 *              Connect to Align                     *
	 *                                                  *
	 ***************************************************/
  { 
    mtc::stages::Connect::GroupPlannerVector planners;
    mtc::stages::Connect::GroupPlannerVector interpolation_planners;
    mtc::stages::ConnectMFReverse::GroupCartPlannerVector carteisan_planners;
    mtc::stages::ConnectMFReverse::GroupPipePlannerVector chomp_planners;
    mtc::stages::Connect::GroupPlannerVector hand_planners;
    planners = {{lead_arm_group_name, lead_sampling_planner}, {follow_arm_group_name, follow_sampling_planner}};
    interpolation_planners = {{lead_arm_group_name, lead_interpolation_planner}, {follow_arm_group_name, follow_interpolation_planner}};
    carteisan_planners = {{lead_arm_group_name, lead_cartesian_planner}, {follow_arm_group_name, follow_cartesian_planner}};
    chomp_planners = {{lead_arm_group_name, lead_chomp_planner}, {follow_arm_group_name, follow_chomp_planner}};
    hand_planners = {{lead_hand_group_name, lead_interpolation_planner}, {follow_hand_group_name, follow_interpolation_planner}};
    
    if (use_cartesian_connect){
      GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};

      moveit::planning_interface::MoveGroupInterfacePtr lead_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, lead_arm_group_name);
      auto stage_move_to_align = std::make_unique<mtc::stages::ConnectMFReverse>("move to align", planners, interpolation_planners, carteisan_planners, chomp_planners,
                                                                                  hand_planners, lead_move_group_interface, visual_tools_);
      stage_move_to_align->setTimeout(10.0);
      stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);
      stage_move_to_align->properties().set("lead_group", lead_arm_group_name);
      stage_move_to_align->properties().set("lead_hand_group", lead_hand_group_name);
      stage_move_to_align->properties().set("lead_base_link", lead_base_frame);
      stage_move_to_align->properties().set("follow_group", follow_arm_group_name);
      stage_move_to_align->properties().set("follow_hand_group", follow_hand_group_name);
      stage_move_to_align->properties().set("follow_base_link", follow_base_frame);
      stage_move_to_align->properties().set("grasp_frame", start_frame_name);
      stage_move_to_align->properties().set("goal_frame", goal_frame_name);
      stage_move_to_align->properties().set("lead_hand_to_tcp_transform", lead_hand_to_tcp_transform_);
      stage_move_to_align->properties().set("follow_hand_to_tcp_transform", follow_hand_to_tcp_transform_);
      stage_move_to_align->properties().set("lead_flange_to_tcp_transform", lead_flange_to_tcp_transform_);
      stage_move_to_align->properties().set("follow_flange_to_tcp_transform", follow_flange_to_tcp_transform_);
      stage_move_to_align->properties().set("quat_clip2ee", quat_clip2ee);
      stage_move_to_align->properties().set("attach_pull_cable", node_->get_parameter("attach_pull_cable").as_bool());
      stage_move_to_align->properties().set("attach_transport_cable", node_->get_parameter("attach_transport_cable").as_bool());
      
      stage_move_to_align->properties().set("leader_grasp_offset_magnitude", leader_grasp_offset_magnitude_);
      stage_move_to_align->properties().set("follower_grasp_offset_magnitude", follower_grasp_offset_magnitude_);

      stage_move_to_align->properties().set("track_offset", desired_ee_distance);
      stage_move_to_align->properties().set("follow_grasp_offset", follower_grasp_offset_magnitude_[1]);
      stage_move_to_align->properties().set("clip_sign", clip_sign);
      stage_move_to_align->properties().set("grasp_clip_size", start_clip_size);
      stage_move_to_align->setEndEffector(ik_endeffectors);

      std::vector<std::string> links = { follow_hand_frame, lead_hand_frame };
      std::vector<Eigen::Isometry3d> tcp_offsets = { follow_hand_to_tcp_transform_, lead_hand_to_tcp_transform_ };
      auto cartesian_cost = std::make_shared<moveit::task_constructor::cost::LinkMotionSum>(links, tcp_offsets);

      std::map<std::string, double> joint_weights = {
        { "right_panda_joint1", 10.0 },
        { "right_panda_joint2",  6.0 },
        { "right_panda_joint3",  4.0 },
        { "right_panda_joint4",  2.0 },
        { "right_panda_joint5",  1.0 },
        { "right_panda_joint6",  1.0 },
        { "right_panda_joint7",  1.0 },

        { "left_panda_joint1", 10.0 },
        { "left_panda_joint2",  6.0 },
        { "left_panda_joint3",  4.0 },
        { "left_panda_joint4",  2.0 },
        { "left_panda_joint5",  1.0 },
        { "left_panda_joint6",  1.0 },
        { "left_panda_joint7",  1.0 },
      };
      auto joint_cost = std::make_shared<moveit::task_constructor::cost::JointRiemannianCost>(joint_weights);

      auto combo = std::make_shared<moveit::task_constructor::cost::WeightedSumTrajectoryCost>();
      combo->add(joint_cost,  /*alpha=*/1.0);
      combo->add(cartesian_cost, /*beta =*/0.1);
      stage_move_to_align->setCostTerm(combo);

      task.add(std::move(stage_move_to_align));

    }else{
      auto stage_move_to_align = std::make_unique<mtc::stages::Connect>("move to align", planners, hand_planners);
      stage_move_to_align->setTimeout(5.0);
      stage_move_to_align->properties().configureInitFrom(mtc::Stage::PARENT);

      // add path constraints
      moveit_msgs::msg::Constraints path_constraints = createBoxConstraints(lead_hand_frame, lead_target_pose, 0.1, 0.1, 0.1);
      stage_move_to_align->setPathConstraints(path_constraints);
      RCLCPP_INFO(LOGGER, "Path constraints set");

      // stage_move_to_align->properties().set("merge_mode", mtc::stages::ConnectMF::MergeMode::SEQUENTIAL);
    
      task.add(std::move(stage_move_to_align));
    }
  }
  
  /****************************************************
	 *                                                  *
	 *               Pick Container                     *
	 *                                                  *
	 ***************************************************/
  {
    auto align = std::make_unique<mtc::SerialContainer>("pick object");

  /****************************************************
  ---- *               Follower Grasping              *
  ***************************************************/
  {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", follow_interpolation_planner);
      stage->setGroup(follow_hand_group_name);
      stage->setGoal("close");

      align->insert(std::move(stage));
  }

      /****************************************************
  ---- *              Dual Insertion in EE-z            *
    ***************************************************/
    if (if_approach){
      mtc::stages::MoveRelativeMultiple::GroupPlannerVector cartesian_planners;
      cartesian_planners = {{follow_arm_group_name, follow_cartesian_planner}, {lead_arm_group_name, lead_cartesian_planner}};

      auto stage =
          std::make_unique<mtc::stages::MoveRelativeMultiple>("insertion", cartesian_planners);
      stage->properties().set("marker_ns", "insertion");
      // stage->properties().set("link", lead_hand_frame);

      GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}};
      GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_flange_to_tcp_transform_}, {lead_arm_group_name, lead_flange_to_tcp_transform_}};
      
      stage->setIKFrame(ik_frame_transforms, ik_hand_frames);
      stage->setGroup({follow_arm_group_name, lead_arm_group_name});
      stage->setMinMaxDistance(0.02, 0.06); // vector is scaled to max distance, and is only accepted if greater than min distance

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec; 
      vec.header.frame_id = goal_frame_name;
      // vec.vector.z = -0.05;
      // vec.vector.z = -(insertion_offset_magnitude_+0.02);
      // vec.vector.x = -(insertion_offset_magnitude_+0.02);
      vec.vector.x = insertion_vector_[0];
      vec.vector.y = insertion_vector_[1];
      vec.vector.z = insertion_vector_[2];
      stage->setDirection(vec);
      // task.add(std::move(stage));
      align->insert(std::move(stage));
    }

  /****************************************************
  ---- *    Generate Target Pose for dual arm transport *
	***************************************************/
    GroupStringDict goal_frames = {{lead_arm_group_name, goal_frame_name}, {follow_arm_group_name, goal_frame_name}};
    
    GroupPoseDict pose_pairs = {{follow_arm_group_name, follow_target_pose}, {lead_arm_group_name, lead_target_pose}};

    std::vector<std::string> ik_groups = {follow_arm_group_name, lead_arm_group_name};
    GroupStringDict ik_endeffectors = {{follow_arm_group_name, follow_hand_group_name}, {lead_arm_group_name, lead_hand_group_name}};
    GroupStringDict ik_hand_frames = {{follow_arm_group_name, follow_hand_frame}, {lead_arm_group_name, lead_hand_frame}, };
    GroupPoseMatrixDict ik_frame_transforms = {{follow_arm_group_name, follow_hand_to_tcp_transform_}, {lead_arm_group_name, lead_hand_to_tcp_transform_}};
    GroupStringDict pre_grasp_pose = {{follow_arm_group_name, "close"}, {lead_arm_group_name, "close"}};

    auto grasp_generator = std::make_unique<mtc::stages::GenerateGraspPoseDual>("generate clipping pose", ik_groups);
    grasp_generator->setEndEffector(ik_endeffectors);
    grasp_generator->properties().set("marker_ns", "align_pose");
    grasp_generator->properties().set("explr_axis", "x");
    grasp_generator->setAngleDelta(0.2); // enumerate over angles from 0 to 6.4 (less then 2 PI)
    grasp_generator->setPreGraspPose(pre_grasp_pose);
    grasp_generator->setGraspPose("close");
    grasp_generator->setObject(goal_frames); // object sets target pose frame
    grasp_generator->setTargetPoseInObject(pose_pairs);
    grasp_generator->setMonitoredStage(pre_move_stage_ptr);
    grasp_generator->properties().set("generate_group", follow_arm_group_name);
    grasp_generator->properties().set("planning_frame", goal_frame_name);

    auto ik_wrapper = std::make_unique<mtc::stages::ComputeIKMultiple>("clipping pose IK", std::move(grasp_generator), ik_groups, dual_arm_group_name);
    ik_wrapper->setSubGroups(ik_groups);
    ik_wrapper->setGroup(dual_arm_group_name);
    ik_wrapper->setEndEffector(ik_endeffectors);
    ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_poses"});
    ik_wrapper->setMaxIKSolutions(10);
    ik_wrapper->setMinSolutionDistance(1.0);
    ik_wrapper->setIKFrame(ik_frame_transforms, ik_hand_frames);

    geometry_msgs::msg::Vector3Stamped insertion_dir_vector; 
    insertion_dir_vector.header.frame_id = goal_frame_name;
    insertion_dir_vector.vector.x = insertion_vector_[0];
    insertion_dir_vector.vector.y = insertion_vector_[1];
    insertion_dir_vector.vector.z = insertion_vector_[2];

    ik_wrapper->properties().set("singularity_threshold", 0.10);

    std::map<std::string,double> manipuability_weights = {{lead_arm_group_name, 1.0}, {follow_arm_group_name, 1.0}};
    std::map<std::string, Eigen::Isometry3d> group_tcp_offsets = {{lead_arm_group_name, lead_hand_to_tcp_transform_}, {follow_arm_group_name, follow_hand_to_tcp_transform_}};

    auto dir_cost = std::make_shared<moveit::task_constructor::cost::DirectionalManipulability>(
        /*group_ee=*/ik_hand_frames,
        /*direction_vec=*/insertion_dir_vector,
        /*space=*/moveit::task_constructor::cost::DirectionalManipulability::Space::TRANSLATION,
        /*group_weights=*/manipuability_weights,
        /*group_tcp_offsets=*/group_tcp_offsets,
        /*epsilon=*/1e-4,
        /*mode=*/moveit::task_constructor::cost::DirectionalManipulability::Mode::AUTO);

    auto manip_vol = std::make_shared<moveit::task_constructor::cost::ManipulabilityVolumeCost>(
        /*group_ee=*/ik_hand_frames,
        /*group_weights=*/manipuability_weights,
        /*translation_only=*/true,
        /*group_tcp_offsets=*/group_tcp_offsets,
        /*lambda=*/1e-4,
        /*mu=*/0.0,
        /*weight=*/1.0);

    auto combo = std::make_shared<moveit::task_constructor::cost::WeightedSumCost>();
    combo->add(dir_cost,  /*alpha=*/2.0);
    combo->add(manip_vol, /*beta =*/2.0);

    ik_wrapper->setCostTerm(combo);

    align->insert(std::move(ik_wrapper));

    task.add(std::move(align));
  }

  return task;
}

mtc::Task MTCTaskNode::createPostTask(std::string& start_frame_name, std::string& goal_frame_name,
                                     bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach, bool clip_added_from_blender)
{
  mtc::Task task;
  task.stages()->setName("post routing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // set target pose
  geometry_msgs::msg::PoseStamped lead_target_pose = createClipGoal(goal_frame_name, leader_pre_insert_offset_);
  geometry_msgs::msg::PoseStamped follow_target_pose = createClipGoal(goal_frame_name, follower_pre_insert_offset_);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // pre_move_stage_ptr = stage_state_current.get();
  }

  // Set up planners
  initializePlanners();

       /****************************************************
  ---- *               Open Hand                      *
	***************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", follow_interpolation_planner);
    stage->setGroup(follow_hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

   /****************************************************
  ---- *               Retrieve in EE-z                *
    ***************************************************/
  {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift", follow_cartesian_planner);
    stage->properties().set("marker_ns", "lift");
    stage->properties().set("link", follow_hand_frame);
    // stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGroup(follow_arm_group_name);
    // stage->setMinMaxDistance(0.1, 0.15); //this will override the moving distance
    // Set hand forward direction
    geometry_msgs::msg::Vector3Stamped vec;
    // vec.header.frame_id = follow_hand_frame;
    vec.header.frame_id = goal_frame_name;
    vec.vector.z = 0.05;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MTCTaskNode::createHomingTask(std::string& start_frame_name, std::string& goal_frame_name, bool if_use_dual, bool if_split_plan, bool if_cartesian_connect, bool if_approach, bool clip_added_from_blender)
{
  mtc::Task task;
  task.stages()->setName("homing task");
  task.loadRobotModel(node_);

  // Initialize robot groups
  initializeGroups();

  // delete markers
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // Current state stage
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_state_current));

    // pre_move_stage_ptr = stage_state_current.get();
  }

  // Set up planners
  initializePlanners();

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move follower back home", follow_sampling_planner);
    stage->setGroup(follow_arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move leader back home", lead_sampling_planner);
    stage->setGroup(lead_arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // Create a publisher to start the follower tracking
  auto tracking_start_pub = mtc_task_node->getNode()->create_publisher<std_msgs::msg::Bool>("/start_tracking", 10);

    // TaskId publisher
  auto taskid_publisher = mtc_task_node->getNode()->create_publisher<std_msgs::msg::String>("/mtc_task_id", rclcpp::QoS(1).transient_local());

  std::vector<std::string> clip_names;

  // Service client to get clip names for update DLO model
  auto clip_names_client = mtc_task_node->getNode()->create_client<moveit_task_constructor_msgs::srv::GetClipNames>("get_clip_names");
  auto request = std::make_shared<moveit_task_constructor_msgs::srv::GetClipNames::Request>();

  // Wait for the service to become available
  while (!clip_names_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(LOGGER, "Waiting for the service...");
  }

  // moveit::planning_interface::MoveGroupInterface move_group(mtc_task_node->getNode(), "right_panda_arm");
  // moveit_visual_tools::MoveItVisualTools visual_tools(mtc_task_node->getNode(), "right_panda_link0", "dual_mtc_routing",
  //                                                     move_group.getRobotModel());
  // visual_tools.loadRemoteControl();

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->initPlanningSceneMonitor();

  // Synchronize with RealWolrd
  bool sync_with_real_world = false;
  if (sync_with_real_world)
  {
    RCLCPP_INFO(LOGGER, "Synchronizing with RealWorld...");
    mtc_task_node->syncwithRealWorld();
    mtc_task_node->getVisualTools().prompt("Synchronization with RealWorld is done. Press 'next' in the RvizVisualToolsGui window to continue the next task");
  }

  // Variables for synchronization
  std::mutex mutex;
  std::condition_variable cv;
  bool response_received = false;

  // List of clip IDs to process
  std::vector<int64_t> clip_id_number_list;
  if (!mtc_task_node->getROSParam("clip_id_number_list", clip_id_number_list)){
    RCLCPP_ERROR(LOGGER, "Failed to get 'clip_id_number_list' parameter. Using default values.");
    clip_id_number_list = {5, 6, 7, 8}; // Default values
  }
  // std::vector<int> clip_id_number_list = {5, 6, 7, 8}; // "clip5", "clip6", "clip7 or 8"
  // std::vector<std::string> clip_ids = {"clip5", "clip6", "clip7", "clip8"}; //"clip5", "clip6", "clip7 or 8"
  std::string initial_clip_id = "clip" + std::to_string(clip_id_number_list[0]);
  
  bool skip_first_clip = false; // Set to true to skip the first clip
  const bool execute_task = true;
  const bool plan_only = false;
  const bool plan_for_dual_arm = true;
  const bool split_task = false;
  const bool use_cartesian_connect = true;
  const bool skip_cartesian_connect = false;
  const bool use_approach = true;
  const bool skip_approach = false;
  const bool clip_not_added_from_blender = false;

  // initial clip
  for (auto i = 0; i < clip_id_number_list.size(); i++)
  {
    auto clip_id_number = clip_id_number_list[i];
    std_msgs::msg::String task_id_msg;
    task_id_msg.data = std::to_string(clip_id_number);
    taskid_publisher->publish(task_id_msg);

    std::string clip_id = "clip" + std::to_string(clip_id_number);
    auto prev_clip_id = (i > 0) ? "clip" +std::to_string(clip_id_number_list[i - 1]) : "";
    // Use visul tools to control the movement from one clip to another
    mtc_task_node->getVisualTools().prompt("[Planning] Press 'next' in the RvizVisualToolsGui window to continue the next task");

    // Update planning scene after execution
    RCLCPP_INFO(LOGGER, "Updating planning scene after MTC execution.");
    mtc_task_node->updatePlanningScene();

    // from the second clip on, set the orientation and add approach
    mtc_task_node->setSelectOrientation(true);

    bool clip_added_from_blender = false;
    mtc_task_node->getROSParam("clip_added_from_blender", clip_added_from_blender);

    if (i>0){
      mtc_task_node->doTask(prev_clip_id, clip_id, execute_task, plan_for_dual_arm, split_task, use_cartesian_connect, use_approach, clip_added_from_blender,
                      [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach, bool clip_from_blender) {
                      return mtc_task_node->createTask(start, goal, dual, split, cartesian, approach, clip_from_blender);
                      });
      
    }else{
      if (!skip_first_clip){
          mtc_task_node->doTask(prev_clip_id, clip_id, plan_only, plan_for_dual_arm, split_task, skip_cartesian_connect, skip_approach, clip_added_from_blender,
            [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach, bool clip_from_blender) {
            return mtc_task_node->createTask(start, goal, dual, split, cartesian, approach, clip_from_blender);
            });
      }
      
    }

    mtc_task_node->getVisualTools().prompt("[Planning] Press 'next' in the RvizVisualToolsGui window to continue the next task");

    // Post task: follower releases and retrieve
    std_msgs::msg::String post_task_id_msg;
    post_task_id_msg.data = std::to_string(clip_id_number) + "_post";
    taskid_publisher->publish(post_task_id_msg);
    if (i == 0 && skip_first_clip)
    {
      // Skip the first clip, so we don't need to create a post task for it
      continue;
    }else{
      mtc_task_node->doTask(prev_clip_id, clip_id, plan_only, plan_for_dual_arm, split_task, skip_cartesian_connect, skip_approach, clip_added_from_blender,
        [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach, bool clip_from_blender) {
        return mtc_task_node->createPostTask(start, goal, dual, split, cartesian, approach, clip_from_blender);
        });
    }

    // save json locally
    mtc_task_node->saveClearanceToJson("planning_clearance_results.json");
    
    clip_names.push_back(clip_id);

    // Send the request
    request->clip_names = clip_names;
    // Send the request asynchronously
    RCLCPP_INFO(LOGGER, "Sending service request...");
    for (const auto& name : clip_names)
    {
      RCLCPP_INFO(LOGGER, "Clip Name: %s", name.c_str());
    }
    clip_names_client->async_send_request(request, 
                            [&](rclcpp::Client<moveit_task_constructor_msgs::srv::GetClipNames>::SharedFuture future) 
    {
      auto response = future.get();
      if (response)
      {
        RCLCPP_INFO(LOGGER, "Service response: %s", response->success ? "true" : "false");
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Service call failed!");
      }

      // Signal that the response has been received
      {
        std::lock_guard<std::mutex> lock(mutex);
        response_received = true;
      }
      cv.notify_one();
    });

    // Wait for the service response before continuing
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [&] { return response_received; });
    }

    // Reset the response flag for the next request
    response_received = false;

    if (clip_id == initial_clip_id)
    {
      // // Use visul tools to control the movement from one clip to another
      mtc_task_node->getVisualTools().prompt("After moving to initial positions, press 'next' in the RvizVisualToolsGui window to start servo");
      // start follower tracking
      std_msgs::msg::Bool start_tracking_msg;
      start_tracking_msg.data = true;
      tracking_start_pub->publish(start_tracking_msg);
    }
  }

  // move back home
  std_msgs::msg::String task_id_msg;
  int homing_id_number = 0; // Homing task ID
  task_id_msg.data = std::to_string(homing_id_number);
  taskid_publisher->publish(task_id_msg);

  std::string clip_id_place_holder = "clip0";
  mtc_task_node->doTask(clip_id_place_holder, clip_id_place_holder, plan_only, plan_for_dual_arm, split_task, skip_cartesian_connect, skip_approach, clip_not_added_from_blender,
  [mtc_task_node](std::string& start, std::string& goal, bool dual, bool split, bool cartesian, bool approach, bool clip_from_blender) {
  return mtc_task_node->createHomingTask(start, goal, dual, split, cartesian, approach, clip_from_blender);
  });


  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
