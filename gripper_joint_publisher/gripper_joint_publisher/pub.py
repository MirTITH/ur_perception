import sensor_msgs
import rclpy
from rclpy.node import Node
import sensor_msgs.msg


class GripperJointPub(Node):
    def __init__(self):
        super().__init__("gripper_joint_publisher")
        self.publisher_ = self.create_publisher(sensor_msgs.msg.JointState, "/joint_states", 10)
        self.sub_ = self.create_subscription(
            sensor_msgs.msg.JointState, "/gripper/joint_states", self.joint_callback, 1
        )
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def joint_callback(self, msg: sensor_msgs.msg.JointState):
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = GripperJointPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
