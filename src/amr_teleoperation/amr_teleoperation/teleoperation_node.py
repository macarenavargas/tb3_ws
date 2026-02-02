import rclpy
from rclpy.node import Node

# Message types
from amr_msgs.msg import Key  # Example message (deprecated)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#from rclpy.qos import QoSPresetProfiles
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np

SAFE_DISTANCE = 0.3 # security distancia detected. 


class MinimalSubscriber(Node):  # Nodes inherit from the base class Node
    def __init__(self) -> None:
        
        super().__init__("teleop")  # Set the node name

        # it starts allowing all of the movements. 
        self._block_forward = False 
        self._block_backward = False 
        self._send_stop_front = True 
        self._send_stop_back = True 

        # Subscribers (beware self._subscriptionsis reserved)
        self._subscriber_key = self.create_subscription(
            msg_type=Key,
            topic="key",
            callback=self._key_callback,
            qos_profile=10,
        )

        self._subscriber_lidar = self.create_subscription(
                    msg_type=LaserScan,
                    topic="scan",
                    callback=self._lidar_callback,
                    qos_profile=qos_profile_sensor_data
                )


        self._publisher = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

    def _key_callback(self, msg: Key) -> None:
        # Log to the terminal with information (info) level
        self.get_logger().info(f"I heard: {msg.key_input}")

        cmd_vel = Twist()

        if msg.key_input == "w": # forward 
            if self._block_forward : 
                cmd_vel.linear.x = 0.0
            else: 
                cmd_vel.linear.x = 0.1
    
        elif msg.key_input == "s": #backward
            if self._block_backward : 
                cmd_vel.linear.x = 0.0
            else: 
                cmd_vel.linear.x = -0.1


        elif msg.key_input == "a": # spin positive 
            cmd_vel.angular.z = 0.5
        elif msg.key_input == "d": #spin negative 
            cmd_vel.angular.z = -0.5
        elif msg.key_input == " ":
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            cmd_vel.linear.z = 0
            cmd_vel.angular.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = 0
        self._publisher.publish(cmd_vel)
        self.get_logger().info(f"Publishing: {cmd_vel.linear.x}, {cmd_vel.angular.z}")

    def _lidar_callback(self, msg:LaserScan) -> None: 
        
        ranges = msg.ranges
        



        # angle 0 : front of the robot 
        # range -> from -15º to +15º 
        sector_size = len(ranges) // 4
        front_distances  = ranges[0 : sector_size] + ranges[3* sector_size :len(ranges)]
        back_distances = ranges[sector_size : 3*sector_size]

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0 # only stops if its published. 


    
        min_front_distance = np.nanmin(front_distances) 
        min_back_distance = np.nanmin(back_distances)
        
        

        if min_front_distance < SAFE_DISTANCE: 
            self._block_forward = True 
            
            if self._send_stop_front: 
                self.get_logger().info("Blocked forward")
                self._publisher.publish(cmd_vel)
                self._send_stop_front = False 
        
        else: 
            self._block_forward = False 
            self._send_stop_front = True 
        

        if min_back_distance < SAFE_DISTANCE: 
            self._block_backward = True 
            
            
            if self._send_stop_back: 
                self.get_logger().info("Blocked backward")
                self._publisher.publish(cmd_vel)
                self._send_stop_back = False 
            
        else: 
            self._block_backward = False  
            self._send_stop_back = True 
    
        

        
        



def main(args=None) -> None:
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly.
    # Optional. Otherwise, it will be done automatically
    # when the garbage collector destroys the node object.
    minimal_subscriber.destroy_node()
    rclpy.shutdown()  # Or rclpy.try_shutdown()


if __name__ == "__main__":
    main()
