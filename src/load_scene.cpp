#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <filesystem>

class ObjectTFBroadcaster
{
public:
    ObjectTFBroadcaster(rclcpp::Node::SharedPtr node) : tf_broadcaster_(node), 
                                                        static_tf_broadcaster_(node),
                                                        tf_buffer_(node->get_clock()), // Initialize TF Buffer with node clock
                                                        tf_listener_(tf_buffer_)      // Initialize TF Listener with TF Buffer
    {
    }

    void publishObjectTF(const std::string& object_name, const geometry_msgs::msg::Pose& object_pose)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = rclcpp::Clock().now();  // Use the node's clock
        transform.header.frame_id = "world";
        transform.child_frame_id = object_name;

        // Set translation
        transform.transform.translation.x = object_pose.position.x;
        transform.transform.translation.y = object_pose.position.y;
        transform.transform.translation.z = object_pose.position.z;

        // Set rotation
        transform.transform.rotation = object_pose.orientation;

        // Broadcast the transform
        // tf_broadcaster_.sendTransform(transform);
        static_tf_broadcaster_.sendTransform(transform);
    }

    geometry_msgs::msg::PoseStamped getPoseTransform(const geometry_msgs::msg::PoseStamped& pose, const std::string& target_frame)
    {
        geometry_msgs::msg::PoseStamped transformed_pose;
        try
        {
            transformed_pose = tf_buffer_.transform(pose, target_frame, tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ObjectTFBroadcaster"), "Failed to transform pose: %s", ex.what());
            throw std::runtime_error("Failed to transform pose");
        }
        return transformed_pose;
    }


private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

void disableCollisions(const std::string &object_name, const std::string &base_name,
                       moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // Prepare the PlanningScene message to update the Allowed Collision Matrix (ACM)
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    // Retrieve the existing ACM from the planning scene
    moveit_msgs::msg::AllowedCollisionMatrix &acm = planning_scene.allowed_collision_matrix;

    // Get the current ACM entry names (links/objects already in the ACM)
    std::vector<std::string> existing_entry_names = planning_scene_interface.getKnownObjectNames();

    // Add the new entries if they are not already in the ACM
    acm.entry_names = existing_entry_names;
    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), object_name) == acm.entry_names.end())
        acm.entry_names.push_back(object_name);
    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), base_name) == acm.entry_names.end())
        acm.entry_names.push_back(base_name);

    // Create a new ACM matrix of size N x N
    size_t matrix_size = acm.entry_names.size();
    acm.entry_values.resize(matrix_size);
    for (auto &row : acm.entry_values)
    {
        row.enabled.resize(matrix_size, false); // Default all collisions to "not allowed"
    }

    // Allow collision for the specific pair (object_name, base_name)
    size_t object_idx = std::distance(acm.entry_names.begin(),
                                      std::find(acm.entry_names.begin(), acm.entry_names.end(), object_name));
    size_t base_idx = std::distance(acm.entry_names.begin(),
                                    std::find(acm.entry_names.begin(), acm.entry_names.end(), base_name));
    acm.entry_values[object_idx].enabled[base_idx] = true;
    acm.entry_values[base_idx].enabled[object_idx] = true;

    // Apply the updated Allowed Collision Matrix
    planning_scene_interface.applyPlanningScene(planning_scene);

    RCLCPP_INFO(rclcpp::get_logger("disable_collisions"), "Disabled collision between '%s' and '%s'", object_name.c_str(), base_name.c_str());
}

bool readPosLine(std::string pos_line, bool is_use_qb_board_coordinate, double &x, double &y, double &z)
{   
    double num_x, num_y, num_z;
    if (!(std::istringstream(pos_line) >> num_x >> num_y >> num_z)){
        return false;
    }else{
        if (is_use_qb_board_coordinate){  
            double unit = 0.305/20; // one board has length of 305mm with 20 units
            double origin_x = 0.48;
            double origin_y = 0.0;
            double sign_x = (num_x >= 0.0) ? 1.0 : -1.0;
            double sign_y = (num_y >= 0.0) ? 1.0 : -1.0;
            x = sign_x * (std::abs(num_x) - 0.5) * unit + origin_x;
            y = sign_y * (std::abs(num_y) - 0.5) * unit + origin_y;
        }else{
            x = num_x;
            y = num_y;
        }
        z = num_z; // z is the height of fixture in mm
        return true;
    }

}

bool convertPosToqboardCoordinate(double x, double y, double z,
                                  double &num_x, double &num_y, double &num_z)
{
    // Board geometry
    const double unit = 0.305 / 20.0;     // 1 grid unit = 0.01525 m
    const double origin_x = 0.48;         // world->board origin (m)
    const double origin_y = 0.0;

    // Map world -> continuous QB index, where integer indices correspond to hole centers.
    // The "+0.5" centers the 0 index at half a unit from the origin, matching your file format.
    const double ix = (x - origin_x) / unit + 0.5;
    const double iy = (y - origin_y) / unit + 0.5;

    // Snap to nearest integer grid cell
    num_x = static_cast<double>(std::llround(ix));
    num_y = static_cast<double>(std::llround(iy));

    // Height: if your world Z is meters, QB wants mm integers (common in your files)
    // If your Z is already mm, just remove the *1000 and rounding.
    num_z = z;
    return true;
}

bool readOrientLine(std::string ori_line, bool is_use_qb_board_coordinate, double &qx, double &qy, double &qz, double &qw)
{
    double num_x, num_y, num_z, num_w;
    if (!(std::istringstream(ori_line) >> num_x >> num_y >> num_z >> num_w)){
        return false;
    }else{
        if (is_use_qb_board_coordinate){  
            double Euler_x, Euler_y, Euler_z;
            Euler_x = num_x * M_PI / 180.0; // Convert to radians
            Euler_y = num_y * M_PI / 180.0; // Convert to radians
            Euler_z = num_z * M_PI / 180.0; // Convert to radians
            // Obtain quaternion from Euler angles
            tf2::Quaternion quat;
            quat.setRPY(Euler_x, Euler_y, Euler_z); // Roll, Pitch, Yaw
            quat.normalize();
            qx = quat.x();
            qy = quat.y();
            qz = quat.z();
            qw = quat.w();
        }else{
            qx = num_x;
            qy = num_y;
            qz = num_z;
            qw = num_w;
        }
        return true;
    }
}

bool convertOrientToqboardCoordinate(double qx, double qy, double qz, double qw,
                                     double &num_x, double &num_y, double &num_z, double &num_w)
{
    // 1) extract yaw [deg]
    tf2::Quaternion quat(qx, qy, qz, qw);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);                 // radians
    double yaw_deg = yaw * 180.0 / M_PI;

    // 2) snap to nearest 30°
    constexpr double STEP = 30.0;
    // use integer rounding to avoid FP drift like 59.999999
    const long k = std::lround(yaw_deg / STEP);
    double snapped = k * STEP;                  // ... now a multiple of 30

    // 3) normalize to the range [-90, 270]
    while (snapped < -90.0) snapped += 360.0;
    while (snapped > 270.0) snapped -= 360.0;

    // 4) write QB format (roll, pitch placeholders; yaw in degrees)
    num_x = 0.0;
    num_y = 0.0;
    num_z = snapped;    // ∈ {-90, -60, ..., 240, 270}
    num_w = 0.0;
    return true;
}


// moveit_msgs::msg::CollisionObject makeMeshCollisionObject(
//     const std::string& id,
//     const std::string& resource_uri,                 // e.g. "package://my_env_pkg/meshes/fixture.stl"
//     const std::string& parent_frame,                 // pose is expressed in this frame (e.g. "world")
//     const geometry_msgs::msg::Pose& pose_in_parent,  // pose of the mesh in parent_frame
//     ObjectTFBroadcaster& tf_broadcaster,
//     const Eigen::Vector3d& scale = Eigen::Vector3d(1.0, 1.0, 1.0),
//     const std::string& world_frame = "world")
// {
//   // Load STL
//   std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(resource_uri, scale));
//   if (!mesh)
//     throw std::runtime_error("Failed to load mesh resource: " + resource_uri);

//   shapes::ShapeMsg shape_msg_any;
//   shapes::constructMsgFromShape(mesh.get(), shape_msg_any);
//   shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg_any);

//   // Transform pose -> world
//   geometry_msgs::msg::PoseStamped src_pose;
//   src_pose.header.frame_id = parent_frame;
//   src_pose.pose = pose_in_parent;

//   geometry_msgs::msg::PoseStamped world_pose =
//       tf_broadcaster.getPoseTransform(src_pose, world_frame);

//   // Build CollisionObject (world-frame)
//   moveit_msgs::msg::CollisionObject co;
//   co.id = id;
//   co.header.frame_id = world_frame;
//   co.meshes.push_back(mesh_msg);
//   co.mesh_poses.push_back(world_pose.pose);
//   co.operation = co.ADD;
//   return co;
// }

inline geometry_msgs::msg::Pose firstPoseOf(const moveit_msgs::msg::CollisionObject& co)
{
  if (!co.primitive_poses.empty()) return co.primitive_poses[0];
  if (!co.mesh_poses.empty())      return co.mesh_poses[0];
  if (!co.plane_poses.empty())     return co.plane_poses[0];
  geometry_msgs::msg::Pose p; p.orientation.w = 1.0; return p; // safe fallback
}

std::vector<moveit_msgs::msg::CollisionObject> loadCustomScene(const std::string &scene_path, const std::string &mesh_path, rclcpp::Node::SharedPtr move_group_node, bool use_qb_board_coordinate)
{
    // planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor);
    // if (!ps)
    // {
    //     throw std::runtime_error("Failed to lock planning scene for writing");
    // }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(move_group_node, "right_panda_arm");

    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "right_panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                      move_group_interface.getRobotModel());

    ObjectTFBroadcaster object_tf_broadcaster(move_group_node);

    if (use_qb_board_coordinate)
    {
        RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Reading pose in QB board coordinate and converting to world coordinate.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Reading pose in world coordinate.");
    }

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::ifstream fin(scene_path);
    if (!fin.is_open())
    {
        throw std::runtime_error("Failed to open scene file: " + scene_path);
        return collision_objects;
    }

    // -------- NEW: prepare an output file to save converted world poses --------
    std::unique_ptr<std::ofstream> fout;
    std::string out_path;
    auto ends_with = [](const std::string& s, const std::string& suffix) -> bool {
    return s.size() >= suffix.size() &&
            s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

    {
        const auto in_path = std::filesystem::path(scene_path);
        const std::string stem = in_path.stem().string(); // filename without extension

        if (use_qb_board_coordinate)
        {
            // reading QB -> converting to WORLD -> write *_world.scene
            if (ends_with(stem, "qb"))
            {
                const std::string base = stem.substr(0, stem.size() - 2);
                out_path = (in_path.parent_path() / (base + "world" + in_path.extension().string())).string();
            }
            else
            {
                out_path = (in_path.parent_path() / (stem + "_world" + in_path.extension().string())).string();
            }
        }
        else
        {
            // reading WORLD -> converting to QB -> write *_qb.scene
            if (ends_with(stem, "world"))
            {
                const std::string base = stem.substr(0, stem.size() - 5);
                out_path = (in_path.parent_path() / (base + "qb" + in_path.extension().string())).string();
            }
            else
            {
                out_path = (in_path.parent_path() / (stem + "_qb" + in_path.extension().string())).string();
            }
        }

        fout = std::make_unique<std::ofstream>(out_path, std::ios::out | std::ios::trunc);
        if (!(*fout))
            throw std::runtime_error("Failed to create output scene file: " + out_path);

        // Harmless header your reader ignores
        (*fout) << "(noname)+\n";
    }

    std::string line;
    while (std::getline(fin, line))
    {
        // Skip invalid lines or empty lines
        if (line.empty() || line[0] == '.' || line.find("(noname)+") != std::string::npos)
            continue;

        if (line[0] == '*') // Object definition starts
        {
            std::string object_name = line.substr(2); // Extract object name after "* "
            RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Loading object: %s", object_name.c_str());

            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = object_name;
            collision_object.header.frame_id = "world"; // Default frame

            // Read the next lines for the object block
            std::string pos_line, ori_line, shape_line, dim_line, unused_line;
            double x, y, z, qx, qy, qz, qw, dx, dy, dz;

            // Read position
            // if (!std::getline(fin, pos_line) || !(std::istringstream(pos_line) >> x >> y >> z))
            if (!std::getline(fin, pos_line) || !readPosLine(pos_line, use_qb_board_coordinate, x, y, z))
                throw std::runtime_error("Invalid position line for object: " + object_name);

            // Read orientation
            // if (!std::getline(fin, ori_line) || !(std::istringstream(ori_line) >> qx >> qy >> qz >> qw))
            if (!std::getline(fin, ori_line) || !readOrientLine(ori_line, use_qb_board_coordinate, qx, qy, qz, qw))
                throw std::runtime_error("Invalid orientation line for object: " + object_name);

            // Skip one line (the "1" marker)
            if (!std::getline(fin, unused_line))
                throw std::runtime_error("Unexpected end of file after orientation for object: " + object_name);

            // Read shape type (assuming it's always "box")
            if (!std::getline(fin, shape_line) || (shape_line != "box" && shape_line != "mesh" && shape_line != "cylinder"))
                throw std::runtime_error("Invalid or unsupported shape type for object: " + object_name);

            // Read dimensions
            if (!std::getline(fin, dim_line))
                throw std::runtime_error("Invalid dimensions line for object: " + object_name);

            dz = 0.0;  // default if missing
            std::istringstream dim_iss(dim_line);
            if (!(dim_iss >> dx >> dy))
                throw std::runtime_error("Invalid dimensions line for object: " + object_name);
            if (!(dim_iss >> dz)) {
                // dz missing, keep default 0.0 (or log a warning if you want)
                // RCLCPP_WARN(rclcpp::get_logger("load_scene"),
                //             "No dz provided for object '%s'; defaulting dz = 0.0",
                //             object_name.c_str());
            }

            // Skip the remaining unused lines in the block
            for (int i = 0; i < 4; ++i)
            {
                if (!std::getline(fin, unused_line))
                    throw std::runtime_error("Unexpected end of file in unused block for object: " + object_name);
            }

            // Create pose
            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.x = qx;
            pose.orientation.y = qy;
            pose.orientation.z = qz;
            pose.orientation.w = qw;

            if (shape_line == "box") {
                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[primitive.BOX_X] = dx;
                primitive.dimensions[primitive.BOX_Y] = dy;
                primitive.dimensions[primitive.BOX_Z] = dz;

                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(pose);
                collision_object.operation = collision_object.ADD;

            } else if (shape_line == "cylinder"){
                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                primitive.dimensions[primitive.CYLINDER_HEIGHT] = dy; // height
                primitive.dimensions[primitive.CYLINDER_RADIUS] = dx / 2.0; // radius

                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(pose);
                collision_object.operation = collision_object.ADD;

            } else if(shape_line == "mesh"){
                // Use dim_line (already parsed into dx, dy, dz) as SCALE
                // Eigen::Vector3d scale(dx, dy, dz);  
                Eigen::Vector3d scale (1, 1, 1); // for STL, use 1,1,1

                // const std::string mesh_path = "/home/tp2/ws_humble/scene/" + object_name + ".stl";
                if (mesh_path.empty()) {
                    throw std::runtime_error("Mesh path is empty for object: " + object_name);
                }
                if (!std::filesystem::exists(mesh_path)) {
                    throw std::runtime_error("Mesh file does not exist: " + mesh_path);
                }
                std::string uri = "file://" + std::filesystem::absolute(mesh_path).string(); // -> file:///home/...

                // auto mesh_co = makeMeshCollisionObject(
                //     object_name, uri, /*parent_frame=*/"world", pose,
                //     object_tf_broadcaster, /*scale=*/scale, /*world_frame=*/"world");

                  // Load STL
                std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(uri, scale));
                if (!mesh)
                    throw std::runtime_error("Failed to load mesh resource: " + uri);

                shapes::ShapeMsg shape_msg_any;
                shapes::constructMsgFromShape(mesh.get(), shape_msg_any);
                shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg_any);

                // Build CollisionObject (world-frame)
                collision_object.meshes.push_back(mesh_msg);
                collision_object.mesh_poses.push_back(pose);
                collision_object.operation = collision_object.ADD;
            }

            collision_objects.push_back(collision_object);

            if (fout)
            {
                if (use_qb_board_coordinate)
                {
                    // Already converted to WORLD by read*Line(..., true) -> write WORLD
                    (*fout) << "* " << object_name << "\n";
                    (*fout) << std::fixed << std::setprecision(9)
                            << x << " " << y << " " << z << "\n";
                    (*fout) << std::setprecision(10)
                            << qx << " " << qy << " " << qz << " " << qw << "\n";
                    (*fout) << "1\n" << shape_line << "\n";
                    (*fout) << std::setprecision(9) << dx << " " << dy << " " << dz << "\n";
                    (*fout) << "0 0 0\n" << "0 0 0 1\n" << "0 0 0 0\n" << "0\n";
                }
                else
                {
                    // Convert WORLD -> QB once
                    double nx, ny, nz, nqx, nqy, nqz, nqw;
                    convertPosToqboardCoordinate(x, y, z, nx, ny, nz);
                    convertOrientToqboardCoordinate(qx, qy, qz, qw, nqx, nqy, nqz, nqw);

                    (*fout) << "* " << object_name << "\n";
                    (*fout) << std::fixed << std::setprecision(9)
                            << nx << " " << ny << " " << nz << "\n";
                    (*fout) << std::setprecision(10)
                            << nqx << " " << nqy << " " << nqz << " " << nqw << "\n";
                    (*fout) << "1\n" << shape_line << "\n";
                    (*fout) << std::setprecision(9) << dx << " " << dy << " " << dz << "\n";
                    (*fout) << "0 0 0\n" << "0 0 0 1\n" << "0 0 0 0\n" << "0\n";
                }
            }


            rclcpp::sleep_for(std::chrono::milliseconds(100));

            // Disable collision with base
            // disableCollisions(object_name, "base", planning_scene_interface);

            // Publish the object's TF
            // publishObjectTF(collision_object.id, pose, move_group_node);
            // object_tf_broadcaster.publishObjectTF(collision_object.id, pose);
        }
    }

    // Add object to planning scene
    planning_scene_interface.addCollisionObjects(collision_objects);
    RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Added %zu collision objects to planning scene", collision_objects.size());

    // publish tf
    for (const auto &collision_object : collision_objects)
    {
        object_tf_broadcaster.publishObjectTF(collision_object.id, firstPoseOf(collision_object));
    }
    RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Published %d object TFs", collision_objects.size());

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools.trigger();

    RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Scene loaded successfully from %s", scene_path.c_str());

    return collision_objects;
}

void loadObjectHats(std::vector<moveit_msgs::msg::CollisionObject> &collision_objects, rclcpp::Node::SharedPtr move_group_node)
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ObjectTFBroadcaster object_tf_broadcaster(move_group_node);

    std::vector<moveit_msgs::msg::CollisionObject> hat_objects;

    for (const auto &collision_object : collision_objects)
    {   
        // only add for objects with name "clip_*"
        if (collision_object.id.find("clip") == std::string::npos)
        {
            continue;
        }
        // get collision obejct's name and size
        std::string object_name = collision_object.id;
        double dx = collision_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
        double dy = collision_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
        double dz = collision_object.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];

        moveit_msgs::msg::CollisionObject hat_object;
        std::string hat_name = object_name + "_hat";

        RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Loading object hat: %s", hat_name.c_str());

        hat_object.id = hat_name;
        hat_object.header.frame_id = object_name; // Use the original object's frame as the base frame

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = dx;
        primitive.dimensions[primitive.BOX_Y] = dy;
        primitive.dimensions[primitive.BOX_Z] = 0.02;

        // Create pose
        geometry_msgs::msg::Pose pose_in_object_frame;

        pose_in_object_frame.position.x = 0;        // pose.position.x = collision_object.primitive_poses[0].position.x + 0.25*dx;
        // pose.position.y = collision_object.primitive_poses[0].position.y + 0.25*dy;
        // pose.position.z = collision_object.primitive_poses[0].position.z + 0.5*dz + 0.01;
        // pose.orientation.x = collision_object.primitive_poses[0].orientation.x;
        // pose.orientation.y = collision_object.primitive_poses[0].orientation.y;
        // pose.orientation.z = collision_object.primitive_poses[0].orientation.z;
        // pose.orientation.w = collision_object.primitive_poses[0].orientation.w;
        pose_in_object_frame.position.y = 0;
        pose_in_object_frame.position.z = 0.5*dz + 0.01;
        pose_in_object_frame.orientation.x = 0.0;
        pose_in_object_frame.orientation.y = 0.0;
        pose_in_object_frame.orientation.z = 0.0;
        pose_in_object_frame.orientation.w = 1.0;

        // Transform pose to world frame
        geometry_msgs::msg::PoseStamped pose_in_world_frame;
        pose_in_world_frame.header.frame_id = "world"; // Set the frame to world
        pose_in_world_frame.pose = pose_in_object_frame;
        pose_in_world_frame = object_tf_broadcaster.getPoseTransform(pose_in_world_frame, "world");

        hat_object.primitives.push_back(primitive);
        hat_object.primitive_poses.push_back(pose_in_world_frame.pose);
        hat_object.operation = hat_object.ADD;

        hat_objects.push_back(hat_object);

        // sleep for a while
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    planning_scene_interface.addCollisionObjects(hat_objects);

    RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Added %d hat objects to planning scene", hat_objects.size());

    // publish tf for hat objects
    for (const auto &hat_object : hat_objects)
    {
        object_tf_broadcaster.publishObjectTF(hat_object.id, hat_object.primitive_poses[0]);
    }
    RCLCPP_INFO(rclcpp::get_logger("load_scene"), "Published %d hat object TFs", hat_objects.size());

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scene_loader");

    if (argc < 4)
    {
        RCLCPP_ERROR(node->get_logger(), "Usage: ros2 run <package_name> <executable_name> <path_to_scene_file>");
        return 1;
    }

    std::string scene_file = argv[1];
    std::string mesh_file = argv[2];
    std::vector<moveit_msgs::msg::CollisionObject> environment_objects;
    try
    {
        environment_objects = loadCustomScene(scene_file, mesh_file, node, false);
        RCLCPP_INFO(node->get_logger(), "Scene loaded successfully!");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error loading scene: %s", e.what());
    }

    // sleep for a while
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // Load clip file if provided
    if (argc > 3 && std::string(argv[3]).empty() == false)
    {
        std::string clip_file = argv[3];
        bool use_qb_board_coordinate = (argc > 4) ? (std::string(argv[4]) == "true") : true;
        RCLCPP_INFO(node->get_logger(), "Clip file provided: %s", clip_file.c_str());
        
        std::vector<moveit_msgs::msg::CollisionObject> clip_objects;
        try
        {
            clip_objects = loadCustomScene(clip_file, "", node, use_qb_board_coordinate);
            RCLCPP_INFO(node->get_logger(), "Clip loaded successfully!");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Error loading clip: %s", e.what());
        }

        bool add_clip_hats = (argc > 5) ? (std::string(argv[5]) == "true") : false;
        // Load clip hats
        if (add_clip_hats)
        {
            try
            {
                loadObjectHats(clip_objects, node);
                RCLCPP_INFO(node->get_logger(), "Clip hats loaded successfully!");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node->get_logger(), "Error loading clip hats: %s", e.what());
            }
        }
    }   
    else
    {
        RCLCPP_WARN(node->get_logger(), "No clip file provided.");
    }

    rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
