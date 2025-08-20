import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading

class Go2JointStateBridge(Node):
    def __init__(self):
        super().__init__('go2_joint_state_bridge')
        self.publisher_ = self.create_publisher(String, '/go2/joint_state_raw', 10)

        # cpp 실행 프로세스 생성
        self.process = subprocess.Popen(
            ['sudo', '/home/unitree/unitree_sdk2-main/build/bin/go2_joint_state_wrapper', 'eth0'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        self.buffer = []  # Joint data 임시 저장
        self.thread = threading.Thread(target=self.read_cpp_output)
        self.thread.daemon = True
        self.thread.start()

    def read_cpp_output(self):
        for line in self.process.stdout:
            if line.strip() == '=== end ===':
                if self.buffer:
                    msg = String()
                    msg.data = "\n".join(self.buffer)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"📤 [Published 12 Joints]\n{msg.data}")
                    self.buffer.clear()
            else:
                self.buffer.append(line.strip())

    def destroy_node(self):
        if self.process.poll() is None:
            self.process.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 KeyboardInterrupt: Node Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
