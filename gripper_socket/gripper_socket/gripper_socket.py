import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import time
import threading


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("gripper_socket")

        self.declare_parameter("remote_host", "")
        self.declare_parameter("remote_port", "")

        self.remote_host = self.get_parameter("remote_host").get_parameter_value().string_value
        self.remote_port = self.get_parameter("remote_port").get_parameter_value().string_value
        assert self.remote_host, "remote_host is not set"
        assert self.remote_port, "remote_port is not set"
        self.get_logger().info(f"remote_host: {self.remote_host}, remote_port: {self.remote_port}")
        self.lock = threading.Lock()

        self.__connect(self.remote_host, self.remote_port, retry=True)
        self.get_logger().info("Connected")

        self.receive_thread = threading.Thread(target=self.__receive_thread)
        self.receive_thread.start()

        self.i: int = 0
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def __receive_thread(self):
        while rclpy.ok():
            try:
                raw_data = self.socket.recv(1024)
                data: dict = json.loads(raw_data)
                self.get_logger().info(f"Received size: {len(raw_data)}. Data: {data}")
            except Exception as e:
                self.get_logger().error(f"{e}")
                self.__connect(self.remote_host, self.remote_port, retry=True)
        
        self.get_logger().info("Thread closed")

    def __connect(self, remote_host, remote_port, retry=True):
        while rclpy.ok():
            try:
                with self.lock:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                remote_ip = socket.gethostbyname(remote_host)
                self.get_logger().info(f"Connecting to {remote_ip}:{int(remote_port)}")
                self.socket.connect((remote_ip, int(remote_port)))
                break
            except Exception as e:
                if not retry:
                    raise e
                self.get_logger().error(f"Error: {e}. Retry in 1 seconds.")
                time.sleep(1)

    def __send(self, msg: dict):
        # pass
        self.socket.sendall(json.dumps(msg).encode())
        self.get_logger().info(f"Sent: {msg}")

    def timer_callback(self):
        while rclpy.ok():
            try:
                msg = {
                    "data": f"Hello World {self.i}",
                    "seq": self.i,
                }
                self.i += 1
                self.__send(msg)
                break
            except Exception as e:
                self.get_logger().error(f"{e}")
                self.__connect(self.remote_host, self.remote_port, retry=True)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()
    minimal_publisher.receive_thread.join()


if __name__ == "__main__":
    main()
