import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TFRelay(Node):

    def __init__(self, namespace, agent):
        super().__init__('tf_relay'+ '_' + str(agent))
        #tf_topic = '/' + str(namespace) + str(agent)+ '/tf'
        tf_topic = '/tf'
        #self.frame_prefix = str(namespace) + str(agent) + '/'
        self.child_frame_prefix = str(namespace) + str(agent)
        self.frame_prefix = 'world'
        self.frame_get = str(namespace)+str(agent)+'/'+'odom'
        self.frame_get2 = str(namespace)+str(agent)+'/'+'sensor_frame'
      
        self.subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_topic,
            callback=self.tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10)
        )
        self.publisher = self.create_publisher(
            msg_type=TFMessage,
            topic='/' + str(namespace) + str(agent)+ '/tf',
            qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE, depth=10))


    def tf_callback(self, msg):
        for transform in msg.transforms:
            #print(transform.header.frame_id)
        
            #if transform.header.frame_id == self.frame_get:
            if transform.header.frame_id == self.frame_get and transform.child_frame_id == self.frame_get2:
     
            
			tmsg = TFMessage()
			tmsg.header.frame_id = self.frame_prefix
			tmsg.child_frame_id = self.frame_prefix
			tmsg.transform.translation.x = transform.transform.translation.x
			print(transform.transform.translation.x)
			print(transform.transform.translation.y)
                	transform.header.frame_id = self.frame_prefix
                	transform.child_frame_id = self.child_frame_prefix 
                	self.publisher.publish(transform)
         
     
