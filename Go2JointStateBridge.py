#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import subprocess
import threading
import re

class Go2JointStateBridge(Node):
    def __init__(self):
        super().__init__('go2_joint_state_bridge')
        
        # ROS 2 Publisher for JointState messages
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_command',
            1
        )

        # Joint names and mapping for standard ROS JointState message
        self.joint_names = [
            'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
            'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
            'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint'
        ]

        # The mapping from Unitree's motor_state index to the desired joint name order.
        # This matches the C++ code's output (0-11) to the joint_names list.
        # Hip: FL, FR, RL, RR
        # Thigh: FL, FR, RL, RR
        # Calf: FL, FR, RL, RR
        self.motor_indices_to_joint_names_order = [
            3, 0, 9, 6,     # Hip joints (FL, FR, RL, RR)
            4, 1, 10, 7,    # Thigh joints
            5, 2, 11, 8     # Calf joints
        ]
        
        # Buffer to store parsed joint data from the C++ process
        self.joint_data_buffer = {}
        
        # Subprocess to run the C++ executable
        self.process = subprocess.Popen(
            ['sudo', '/home/unitree/unitree_sdk2-main/build/bin/go2_joint_state_wrapper', 'eth0'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Regex to parse the C++ output lines
        self.line_regex = re.compile(r'Joint (\d+) -> q: ([-+]?\d*\.\d+|\d+), dq: ([-+]?\d*\.\d+|\d+)')

        # Start a thread to read the stdout of the C++ process asynchronously
        self.thread = threading.Thread(target=self.read_cpp_output)
        self.thread.daemon = True
        self.thread.start()
        
        self.get_logger().info('Go2JointStateBridge node started. Waiting for C++ process output...')

    def read_cpp_output(self):
        """
        Reads output from the C++ subprocess, parses it, and publishes a JointState message.
        """
        try:
            for line in iter(self.process.stdout.readline, ''):
                line = line.strip()
                if not line:
                    continue

                if line == "=== end ===":
                    # When a full data set is received, process and publish it.
                    if len(self.joint_data_buffer) == 12:
                        self.publish_joint_state()
                        self.joint_data_buffer = {}
                    else:
                        self.get_logger().warn(f"Received '=== end ===' with incomplete data. Buffer size: {len(self.joint_data_buffer)}")
                    continue
                
                # Parse the line using regex
                match = self.line_regex.search(line)
                if match:
                    joint_index = int(match.group(1))
                    q = float(match.group(2))
                    dq = float(match.group(3))
                    
                    self.joint_data_buffer[joint_index] = {'q': q, 'dq': dq}
                else:
                    self.get_logger().debug(f"Could not parse line: {line}")

        except Exception as e:
            self.get_logger().error(f"Error reading C++ output: {e}")

    def publish_joint_state(self):
        """
        Constructs and publishes the JointState message from the buffered data.
        """
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        
        # Populate position and velocity lists in the correct order
        joint_msg.position = []
        joint_msg.velocity = []
        
        for motor_idx in self.motor_indices_to_joint_names_order:
            if motor_idx in self.joint_data_buffer:
                joint_msg.position.append(self.joint_data_buffer[motor_idx]['q'])
                joint_msg.velocity.append(self.joint_data_buffer[motor_idx]['dq'])
            else:
                # Handle missing data points gracefully
                self.get_logger().warn(f"Missing data for motor index {motor_idx}. Publishing partial message.")
                return

        # Publish the message
        self.joint_pub.publish(joint_msg)
        self.get_logger().info('Published complete JointState message to Isaac Sim')

    def destroy_node(self):
        """
        Terminates the C++ subprocess before the node is destroyed.
        """
        if self.process.poll() is None:
            self.process.terminate()
            self.get_logger().info("Terminated go2_joint_state_wrapper process.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
