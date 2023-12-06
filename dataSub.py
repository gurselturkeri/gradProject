import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('motorDataSubscriber')
        self.currentPoseSub = self.create_subscription(Int64,'currentPose',self.currentPoseCB,10)
        self.desiredPoseSub = self.create_subscription(Int64,'desiredPose',self.desiredPoseCB,10)
    

    def currentPoseCB(self, current_msg):
        self.get_logger().info('Current Position Data: "%s"' % current_msg.data)


    def desiredPoseCB(self, desired_msg):
        self.get_logger().info('Desired Position Data: "%s"' % desired_msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()