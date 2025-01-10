

import threading
import time

from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

import math
import numpy as np
from turtlebot4_msgs.msg import UserLed
from geometry_msgs.msg import Twist

from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


#pose.orientation is a quaternion, this converts it to an angle which represents the global heading of the robot
def quaternion_to_heading_angle(q):
    """
    Finds the heading angle of the robot from its quaternion orientation
    :param q: quaternion rotation from ROS msg
    :return:
    """
    return np.arctan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))
class AgentController(Node):
    def __init__(self,i,neighbors,namespace,mode = 0):
        super().__init__('AgentController')
        self.agent_name = 'robot'+str(i)
        self.namespace= namespace
        self.id = i
        self.neighbors= neighbors
        self.X = dict.fromkeys(neighbors)
        self.trackedNeighbors = dict.fromkeys(neighbors)
        self.subscribers = dict.fromkeys(neighbors)
        self.rot_vel = 0.7
        self.linear_vel = 1.0
        self.neighbor_headings = dict.fromkeys(neighbors)
        self.timer = self.create_timer(0.2,self.on_timer)
        self.publisher = self.create_publisher(Twist, '/'+self.agent_name+'/cmd_vel',1)
        self.create_subscribers()

    # Dock subscription callback
    def dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked
    def create_subscribers(self):
    #Creates subscribers to each neighbors pose.  Sends the pose and the neighbor id to the pose_callback function to handle the data.    
    #################################################################################################################################
        #If Mocap mode:
        if self.mode == 0:       
            for neighbor in self.neighbors:
                neighbor_name = self.namespace+str(neighbor)
                neighbor_name = neighbor_name.replace('ro','turtle')
                print(self.agent_name+' has neighbor: '+neighbor_name)
                try:
                    pose_subscriber = self.create_subscription(
                        PoseStamped,
                        '/vrpn_mocap/'+neighbor_name+'/pose',
                        lambda msg,name = neighbor :self.pose_callback(msg,name),
                        qos_profile_sensor_data)
                    self.subscribers[neighbor] = pose_subscriber
                    print("Adding subscriber:")
                    print("  topic: ", '/vrpn_mocap/'+neighbor_name+'/pose')
                    
                except:
                    print('no subscription was made.')   
        if self.mode == 1:
             for neighbor in self.neighbors:
                neighbor_name = self.namespace+str(neighbor)
                try:
                    pose_subscriber = self.create_subscription(
                        TFMessage,
                        '/tf/'+neighbor_name+'/pose',
                        lambda msg,name = neighbor :self.pose_callback(msg,name),
                        qos_profile_sensor_data)
                    self.subscribers[neighbor] = pose_subscriber
                    print("Adding subscriber:")
                    print("  topic: ", '/vrpn_mocap/'+neighbor_name+'/pose')
                except:
                    print('no subscription was made.')
        if self.mode == 3:
             for neighbor in self.neighbors:
                neighbor_name = self.namespace+str(neighbor)
                try:
                    pose_subscriber = self.create_subscription(
                        TFMessage,
                        '/neighbor_name/tf',
                        lambda msg,name = neighbor :self.pose_callback(msg,name),
                        qos_profile_sensor_data)
                    self.subscribers[neighbor] = pose_subscriber
                    print("Adding subscriber:")
                    print("  topic: ", '/'+neighbor_name+'/tf')
                except:
                    print('no subscription was made.')                        
    def pose_callback(self, msg,neighbor):
    # For each neighbor's pose, stores the (x,y) coordinates in self.X[neighbor]. If the neighbor is this agent, store the agent heading as "self.heading"
    #This callback runs every time a pose is published to the agent's subscribers, so it continuously updates the positions of neighbors.
    ####################################################################################################################################
        """ callback function to get the pose from mocap data """
        # If mocap mode:
        if self.mode == 0:
            x, y = msg.pose.position.x, msg.pose.position.y
            self.neighbor_headings[neighbor] = quaternion_to_heading_angle(msg.pose.orientation)
        #If sim mode:
        elif self.mode ==1:
            x,y = msg.pose.pose.position.x, msg.pose.pose.position.y
        elif self.mode ==3:
            x,y = msg.transform.translation.x,msg.transform.translation.y
        if neighbor == self.id:
            if self.mode ==0:
                self.heading = quaternion_to_heading_angle(msg.pose.orientation)
            elif self.mode ==1:
                self.heading = quaternion_to_heading_angle(msg.pose.pose.orientation)
        self.X[neighbor] = np.array((x,y))  
    # Set User LEDs for TurtleBot 4
    def setLed(self, led, color, period, duty):
        msg = UserLed()
        msg.led = led
        msg.color = color
        msg.blink_period = period
        msg.duty_cycle = duty

        self.user_led_pub.publish(msg)

    # Undock action
    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            print('Undocking failed')

    # Calculate direction of leader relative to agent

    def on_timer(self):
    # Periodic function that checks if agent has the positions of all of its neighbors.  If true, then 'self.Controller' is called. 
    ####################################################################################################################################
        tracking = True
        for neighbor in self.neighbors:
            if self.X[neighbor] is None:
                tracking = False
            else:
                self.trackedNeighbors[neighbor]=1
        if tracking:
            self.Controller()
                   
#------------ CONTROLLER
#------------ X[k] is the state of robot{k}
#------------ Xi is the state of the controlled robot
#------------ Xcnext = The desired next state of current robot
#------------
#------------ Consensus Algorithm:
    def Controller(self):
        #Xi = self.X[self.id]
        Xi = np.array((0.0,0.0))
        target = np.array((0.0,0.0))

        count = 0
        for neighbor in self.neighbors:
          
            if neighbor != self.agent and self.trackedNeighbors[neighbor] ==1:
                Xn = self.X[neighbor]

                target+= Xn-Xi
                count+=1
        if count>0:
             dx = dx/count
             
       
        self.stateUpdate(dx)

#------------STATE UPDATE
#------------Need to determine how best to get a displacement of 'dx' through one twist msg..
#------------Current method: 'rotate then move straight'

    def stateUpdate(self, dx, target_threshold = 0.1):
        msg = Twist()
        x= dx[0]
        y= dx[1]
        if math.sqrt(dx[0]**2+dx[1]**2)< target_threshold:
            print(self.agent+' has reached target position '+str(dx))
        else:
            theta = math.atan2(y,x)
            dtheta  = theta-self.heading
            if theta > 0.2:
                msg.angular.z = self.rot_vel*theta
                msg.linear.x = 0.0
                print('theta'+str(theta))
                self.publisher.publish(msg)
            elif theta <-0.2:
                msg.angular.z = theta
                msg.linear.x = 0.0
                print('theta'+str(theta))
                self.publisher.publish(msg)
            else: 
                print(self.agent+' Moving'+str(x) +str(y))   
                msg.angular.z =self.rot_vel*theta
                msg.linear.x = self.linear_vel*math.sqrt(x**2+y**2)
                self.publisher.publish(msg)
