import time
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import Data
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

import math_utils
from project_interfaces.action import Patrol
from project_interfaces.action import Polling

from sim_utils import EventScheduler

WORLD_NAME = "iot_project_world"
MIN_ALTITUDE_TO_PERFORM_PATROL = 4
SIZE = 10

DEBUG_RX = True
DEBUG_SETUP = False

class BalloonController(Node):

    def __init__(self):
        super().__init__("drone_controller")

        #self.timers = []
        self.cache = []
        self.cache_size = SIZE

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )
        
        self.position = Point(x = 0.0, y = 0.0, z = 0.0)
        self.yaw = 0

        self.stop_msg = Twist()
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        

        self.odometry_subscrber = self.create_subscription(
            Odometry,
            'odometry',
            self.store_position,
            10
        )
        
        self.rx_data = self.create_subscription(
            Data,
            'rx_data',
            self.rx_callback,
            10
        )

        self.patrol_action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            self.execute_patrol_action
        )

        self.polling_action_server = ActionServer(
            self,
            Polling,
            'polling',
            self.execute_polling_action
        )

    def rx_callback(self, msg : Data):

        #print the cache length
        #self.get_logger().info(f'{len(self.cache)}')
        
        if len(self.cache)>=self.cache_size: 
            self.remove_LRU()
        self.cache.append(msg)
        self.event_scheduler.schedule_event(msg.duration, self.expire_callback,False,args = [msg])
        


        #self.get_logger().info(f'TIME:(sec:{msg.timestamp.sec},nanosec:{msg.timestamp.nanosec}), DATA: {msg.data}')
        if DEBUG_RX :
            self.get_logger().info('Message received')
            self.print_cache()

    def remove_FIFO(self):
        temp_msg=self.cache[0]
        try:
            self.cache.pop(0)
            if DEBUG_RX :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if DEBUG_RX :self.get_logger().info(f'ERR_FIFO: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass
            
    def remove_RND(self):
        temp_msg=self.cache[random.randint(0, len(self.cache) - 1)]

        try:
            self.cache.remove(temp_msg)
            if DEBUG_RX :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if DEBUG_RX :self.get_logger().info(f'ERR_RND: {temp_msg.sensor_id}-{temp_msg.sqn}')       
            pass

    def remove_LRU(self):
        temp_msg=None
        for m in self.cache:
            if temp_msg==None:
                temp_msg=m
            elif m.timestamp.sec<temp_msg.timestamp.sec or (m.timestamp.sec==temp_msg.timestamp.sec and m.timestamp.nanosec<temp_msg.timestamp.nanosec):
                temp_msg=m
        try:

            self.cache.remove(temp_msg)
            if DEBUG_RX :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if DEBUG_RX :self.get_logger().info(f'ERR_LRU: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass

    def expire_callback(self,msg):
        try:
            self.cache.remove(msg)
            if DEBUG_RX :self.get_logger().info(f'Expired: {msg.sensor_id}-{msg.sqn}')
        except ValueError:
            if DEBUG_RX :self.get_logger().info(f'ERR_EXP: {msg.sensor_id}-{msg.sqn}')
            pass

        #self.get_logger().info(f'{self.cache}')
    def print_cache(self):
        self.get_logger().info('#########################################################')
        for log in self.cache:
           self.get_logger().info(f'{log.sensor_id}-{log.sqn}==>(sec:{log.timestamp.sec},nanosec:{log.timestamp.nanosec})')
        self.get_logger().info('##############################################################')

    def store_position(self, odometry_msg : Odometry):

        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )
        #self.get_logger().info(f"Storing position {self.drone_position}")
    
    def execute_patrol_action(self, goal : ServerGoalHandle):


        command_goal : Patrol.Goal = goal.request

        if DEBUG_SETUP:self.get_logger().info(f"Action requested. Performing movement to targets:\n\t{command_goal.targets}")

        self.fly_to_altitude(MIN_ALTITUDE_TO_PERFORM_PATROL)

        goal.succeed()

        result =  Patrol.Result()
        result.result = "Movement completed"

        return result
    
    def execute_polling_action(self, goal : ServerGoalHandle):

        self.get_logger().info(f'Executing goal, polling sensor {goal.request.sensor_id}')
        
        request_sensor = goal.request.sensor_id

        sensor_data = None
        for d in self.cache:
            if d.sensor_id == request_sensor:
                #self.get_logger().info(f'Data found: {d.sensor_id}-{d.sqn}')
                sensor_data = d
                d.timestamp = self.get_clock().now().to_msg()
            else:
                #self.get_logger().info(f'Data NOT found for sensor {goal.request.sensor_id}')
                sensor_data = None
        
        goal.succeed()
        
        result = Polling.Result()
        if sensor_data:
            result.result = sensor_data
        else:
            result.result.timestamp = self.get_clock().now().to_msg()
            result.result.duration = 4
            result.result.sensor_id = request_sensor
            result.result.sqn = -1
            result.result.data = "404"

        return result

    def fly_to_altitude(self, altitude):

        # Instantiate the move_up message
        move_up = Twist()
        move_up.linear = Vector3(x=0.0, y=0.0, z=1.0)
        move_up.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.cmd_vel_publisher.publish(move_up)
        if DEBUG_SETUP:self.get_logger().info(f'{altitude}')
        # Wait for the drone to reach the desired altitude
        while(self.position.z < altitude):
            time.sleep(0.1)

        # Stop movement after the altitude has been reached
        stop_mov = Twist()
        stop_mov.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_mov.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_mov)



def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor()

    drone_controller = BalloonController()
    executor.add_node(drone_controller)

    executor.spin()

    drone_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
