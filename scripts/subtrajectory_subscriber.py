#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_task_constructor_msgs.msg import Solution, SubTrajectory  
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Int32, String
import numpy as np
import socket
import os
import pickle
import time
import argparse
import json


def default_workspace_dir():
    workspace_dir = os.environ.get("MTC_WORKSPACE_DIR")
    if workspace_dir:
        return workspace_dir
    return os.path.expanduser("~/ws_humble")


class SubTrajectorySubscriber(Node):
    def __init__(self, central_plan_ip, left_mios_ip, left_mios_traj_port, left_mios_tcp_traj_port, right_mios_ip, right_mios_traj_port, right_mios_tcp_traj_port, workspace_dir):
        super().__init__('subtrajectory_subscriber')

        # Create a subscriber and subscribe to the /solution topic
        self.subtrajectory_subscription = self.create_subscription(
            SubTrajectory,  # msg type
            '/mtc_sub_trajectory',  # Topic name
            self.subtraj_callback,  
            10  # Queue size
        )
        self.subtrajectory_subscription  # Preventing garbage collection

        self.task_id = "0"  # Default value until received from topic
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.task_id_sub = self.create_subscription(
            String,
            '/mtc_task_id',
            self.task_id_callback,
            qos_profile
        )
        
        self.central_plan_ip = central_plan_ip
        self.left_mios_ip = left_mios_ip
        self.right_mios_ip = right_mios_ip
        self.left_mios_traj_port = left_mios_traj_port
        self.right_mios_traj_port = right_mios_traj_port
        self.left_mios_tcp_traj_port = left_mios_tcp_traj_port
        self.right_mios_tcp_traj_port = right_mios_tcp_traj_port
        self.workspace_dir = workspace_dir
        self.follower_output_dir = os.path.join(self.workspace_dir, "trajectories_follower")
        self.leader_output_dir = os.path.join(self.workspace_dir, "trajectories_leader")

        self.get_logger().info(f"SubTrajectory Subscriber initialized. Central IP: {self.central_plan_ip}, Left MIOS IP: {self.left_mios_ip}:{self.left_mios_traj_port}, Right MIOS IP: {self.right_mios_ip}:{self.right_mios_traj_port}")
        
    def send_ack(self, ack_port=12345):
        """Send an ACK message to 192.168.1.12 via UDP."""
        ack_ip = '192.168.1.12'
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Instantiate socket
            data_ack = {'plan_done': True}          
            ack_message = json.dumps(data_ack)
            client_socket.sendto(ack_message.encode('utf-8'), (ack_ip, ack_port))
            self.get_logger().info("Sent ACK message.")
            time.sleep(1)
            client_socket.close()
        except Exception as e:
            self.get_logger().error(f"Failed to send ACK: {e} at {ack_ip}:{ack_port}")

    def write_trajectory_udp(self, path, host, port, traj_pos, traj_vel, traj_time, stage_id, traj_id):
        os.makedirs(path, exist_ok=True)
        joint_file_name = 'real_world_task_' + self.task_id + '_stage_' + str(stage_id) + '_traj_'+str(traj_id)+'.txt'
        joint_file_path = os.path.join(path, joint_file_name)

        if host == "local":
            # save the trajectory to local file
            with open(joint_file_path, "w") as f:
                for i in range(len(traj_time)):
                    # Write time
                    f.write(f"{traj_time[i]} ")
                    # Write positions
                    f.write(" ".join(map(str, traj_pos[i])) + " ")
                    # Write velocities
                    f.write(" ".join(map(str, traj_vel[i])) + "\n")
        else:
            client_socket = socket.socket()  # instantiate
            client_socket.connect((host, port))
            self.get_logger().info(f"Connect to robot")
            time.sleep(1)
        
            command = "write_moveit"
            client_socket.send(f"{command}".encode('utf-8'))
            time.sleep(1)
            # send the name of the file
            # joint_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'smooth_real_world_traj_'+str(traj_id)+'.txt')
            # joint_file_name = os.path.basename(joint_file_path)
            
            
            # client_socket.send(f"{os.path.basename(joint_file_path)}".encode())
            # self.get_logger().info(f"send fbasename(joint_file_path)ile name to mios at {server_host}")
            client_socket.send(f"{joint_file_name}".encode('utf-8'))  
            time.sleep(1)
            self.get_logger().info(f"Sent file name: {joint_file_name}")
            
            # send the trajectory
            trajectory_data = {
                "time": traj_time.tolist(),
                "positions": traj_pos.tolist(),
                "velocities": traj_vel.tolist(),
            }
            # Serialize the data using pickle
            serialized_data = pickle.dumps(trajectory_data)

            client_socket.sendall(serialized_data)
            time.sleep(1)
            self.get_logger().info(f"send trajectory to mios at {host}")
            client_socket.close()

    def write_tcp_trajectory_udp(self, path, host, port, stage_id, traj_id):
        os.makedirs(path, exist_ok=True)
        if host == "local":
            self.get_logger().info("Local host specified, skipping TCP trajectory send.")
            return

        tcp_file_name = 'clip' + self.task_id + '_stage_' + str(stage_id) + "_tcp_trajectory_" + str(traj_id) + '.txt'
        tcp_file_path = os.path.join(path, tcp_file_name)
        if not os.path.exists(tcp_file_path):
            self.get_logger().warn(f"TCP trajectory file not found: {tcp_file_path}")
            return

        # Read and parse TCP file
        time_list = []
        tcp_matrix_list = []

        with open(tcp_file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 17:
                    self.get_logger().warn(f"Invalid line in TCP file: {line}")
                    continue
                time_val = float(parts[0])
                matrix_vals = list(map(float, parts[1:]))

                # Convert flat list to 4x4 numpy matrix (column-major)
                matrix = np.array(matrix_vals).reshape((4, 4), order='F')
                time_list.append(time_val)
                tcp_matrix_list.append(matrix)

        # Prepare data for sending
        tcp_data = {
            "time": time_list,
            "transforms": [m.tolist() for m in tcp_matrix_list]  # Serialize as list of lists
        }

        try:
            client_socket = socket.socket()
            client_socket.connect((host, port))
            self.get_logger().info(f"Connected to {host} to send TCP trajectory")

            # Send command
            command = "write_tcp"
            client_socket.send(command.encode('utf-8'))
            time.sleep(1)

            # Send the file name
            client_socket.send(tcp_file_name.encode('utf-8'))
            time.sleep(1)

            # Send the serialized data
            serialized_data = pickle.dumps(tcp_data)
            client_socket.sendall(serialized_data)
            self.get_logger().info(f"Sent TCP trajectory to {host}")
            time.sleep(1)
            client_socket.close()

        except Exception as e:
            self.get_logger().error(f"Error sending TCP trajectory to {host}:{port} -> {e}")

    
    def task_id_callback(self, msg):
        self.task_id = msg.data
        self.get_logger().info(f"[TASK ID] Updated to: {self.task_id}")

    def subtraj_callback(self, msg):
        """Callback function when receiving /subtrajectory message"""
        
        # Get the joint names and points in the trajectory
        traj_joint_names = msg.trajectory.joint_trajectory.joint_names
        traj_points = msg.trajectory.joint_trajectory.points

        if not traj_joint_names:
            self.get_logger().info("Received empty subtrajectory id %d" % msg.info.id)
            return
        else:
            self.get_logger().info("Received subtrajectory id %d for stage %d with %d waypoints" 
                               % (msg.info.id, msg.info.stage_id, len(msg.trajectory.joint_trajectory.points)))
            
            left_indices = []
            right_indices = []
            for i, joint_name in enumerate(traj_joint_names):
                if "finger_joint" in joint_name:
                    # skip the finger trajectory
                    self.get_logger().info(f"Skip finger joint trajectory: {joint_name}")
                    continue
                elif "left" in joint_name:
                    left_indices.append(i)
                elif "right" in joint_name:
                    right_indices.append(i)
                else:
                    self.get_logger().warn(f"Unknown joint name: {joint_name}")
                

        # Extract joint trajectory points
        self.get_logger().info(f"Trajectory Joint names: {', '.join(traj_joint_names)}")
        left_arm_pos_list = []
        right_arm_pos_list = []

        left_arm_vel_list = []
        right_arm_vel_list = []

        time_list = []

        finger_traj_list = []

        for point_idx, point in enumerate(traj_points):
            # Extract time
            time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            time_list.append(time)

            # Extract positions and velocities for left and right arms
            left_arm_pos_list.append([point.positions[i] for i in left_indices])
            right_arm_pos_list.append([point.positions[i] for i in right_indices])

            left_arm_vel_list.append([point.velocities[i] for i in left_indices])
            right_arm_vel_list.append([point.velocities[i] for i in right_indices])

        # Convert to numpy arrays
        left_pos = np.array(left_arm_pos_list)
        right_pos = np.array(right_arm_pos_list)
        left_vel = np.array(left_arm_vel_list)
        right_vel = np.array(right_arm_vel_list)
        time_array = np.array(time_list)
                
        if len(left_indices) > 0:
            self.write_trajectory_udp(self.follower_output_dir, self.left_mios_ip, self.left_mios_traj_port, left_pos, left_vel, time_array, msg.info.stage_id, msg.info.id)
            self.write_tcp_trajectory_udp(self.follower_output_dir, self.left_mios_ip, self.left_mios_tcp_traj_port, msg.info.stage_id, msg.info.id)
            # self.send_ack(ack_port=5085)
            
        if len(right_indices) > 0:
            self.write_trajectory_udp(self.leader_output_dir, self.right_mios_ip, self.right_mios_traj_port, right_pos, right_vel, time_array, msg.info.stage_id, msg.info.id)
            self.write_tcp_trajectory_udp(self.leader_output_dir, self.right_mios_ip, self.right_mios_tcp_traj_port, msg.info.stage_id, msg.info.id)
            # self.send_ack(ack_port=5086)                       

                # if msg_to_mios.robot_id:
                #     msg_to_mios.robot_id.append(robot_id)
                # else:
                #     msg_to_mios.robot_id = [robot_id]
                # if msg_to_mios.traj_id:
                #     msg_to_mios.traj_id.append(traj_id)
                # else:
                #     msg_to_mios.traj_id = [traj_id]
                # client_socket.connect((server_host, server_port))
                # send the write or read command


def main(args=None):
    parser = argparse.ArgumentParser(description='Dual Joint SubTrajectory Publisher')
    # add boolean flag to connect to UR robot or not
    parser.add_argument('--ur_connected', action='store_true', help='Flag to connect to UR robot or not')
    parser.add_argument('--central_ip', default='10.157.175.222', type=str, help='IP address of the central planning computer')
    parser.add_argument('--left_mios_ip', default='10.157.174.87', type=str, help='IP address of the follow mios computer')
    parser.add_argument('--left_mios_traj_port', default=12345, type=int, help='Port of the left mios computer')
    parser.add_argument('--left_mios_tcp_traj_port', default=12345, type=int, help='TCP Port of the left mios computer for trajectory')
    parser.add_argument('--right_mios_ip', default='10.157.174.97', type=str, help='IP address of the lead mios computer')
    parser.add_argument('--right_mios_traj_port', default=12345, type=int, help='Port of the right mios computer')
    parser.add_argument('--right_mios_tcp_traj_port', default=12345, type=int, help='TCP Port of the right mios computer for trajectory')
    parser.add_argument('--workspace_dir', default=default_workspace_dir(), type=str, help='Workspace root used to resolve trajectory output folders')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)

    solution_subscriber = SubTrajectorySubscriber(parsed_args.central_ip, 
                                                  parsed_args.left_mios_ip, parsed_args.left_mios_traj_port, parsed_args.left_mios_tcp_traj_port,
                                                  parsed_args.right_mios_ip, parsed_args.right_mios_traj_port, parsed_args.right_mios_tcp_traj_port,
                                                  parsed_args.workspace_dir)

    rclpy.spin(solution_subscriber)

    solution_subscriber.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
