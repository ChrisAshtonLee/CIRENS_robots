
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TFRelay(Node):

    def __init__(self, namespace, agent):
        super().__init__('tf_relay' + '_' + str(agent))
        tf_topic = '/tf'
        self.child_frame_prefix = str(namespace) + str(agent)
        self.frame_prefix = 'world'
        self.frame_get = str(namespace) + str(agent) + '/' + 'odom'
        self.frame_get2 = str(namespace) + str(agent) + '/' + 'sensor_frame'

        self.subscription = self.create_subscription(
            TFMessage,
            tf_topic,
            self.tf_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        )
        self.publisher = self.create_publisher(
            TFMessage,
            '/' + str(namespace) + str(agent) + '/tf',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == self.frame_get and transform.child_frame_id == self.frame_get2:
                new_transform = TransformStamped()
                new_transform.header.frame_id = self.frame_prefix
                new_transform.child_frame_id = self.child_frame_prefix
                new_transform.transform.translation.x = transform.transform.translation.x
                new_transform.transform.translation.y = transform.transform.translation.y
                new_transform.transform.translation.z = transform.transform.translation.z
                new_transform.transform.rotation = transform.transform.rotation

                new_msg = TFMessage(transforms=[new_transform])
                self.publisher.publish(new_msg)
                print(transform.transform.translation.x)
                print(transform.transform.translation.y)
