import os
import time
import random
import rclpy
import rclpy.action
import math_utils

from dotenv import load_dotenv
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from project_interfaces.msg import Data
from project_interfaces.action import Patrol
from project_interfaces.action import Polling

from sim_utils import EventScheduler

load_dotenv()

WORLD_NAME = "iot_project_world"
MIN_ALTITUDE_TO_PERFORM_PATROL = 4
SIZE = 10

debug_rx = int(os.getenv('DEBUG_RX'))
debug_setup = int(os.getenv('DEBUG_SETUP'))
debug_polling = int(os.getenv('DEBUG_POLLING'))
caching_algorithm = int(os.getenv('CACHING_ALGORITHM'))


class BalloonController(Node):

    def __init__(self):
        super().__init__("drone_controller")

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
        
        rx_callback_group = MutuallyExclusiveCallbackGroup()
        self.rx_data = self.create_subscription(
            Data,
            'rx_data',
            self.rx_callback,
            10,
            callback_group=rx_callback_group
        )

        self.patrol_action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            self.execute_patrol_action
        )

        polling_callback_group = MutuallyExclusiveCallbackGroup()
        self.polling_action_server = ActionServer(
            self,
            Polling,
            'polling',
            self.execute_polling_action,
            goal_callback=self.polling_goal_callback,
            cancel_callback=self.cancel_polling_callback,
            callback_group=polling_callback_group
        )

    def rx_callback(self, msg : Data):

        # Updating the message of the same sensor if present
        temp_msg = None

        for m in self.cache:

            if (temp_msg == None) and (m.sensor_id == msg.sensor_id):
                temp_msg = m
                break

        if temp_msg != None:

            for m in self.cache:
                if temp_msg != None and m.sensor_id == msg.sensor_id and m.timestamp.sec < temp_msg.timestamp.sec or (m.timestamp.sec == temp_msg.timestamp.sec and m.timestamp.nanosec < temp_msg.timestamp.nanosec):
                    temp_msg = m

            self.remove_SPECIFIC(temp_msg)
        
        elif len(self.cache)>=self.cache_size:

            # Idea: Depending on the global variable we apply a different policy for the removal of messages in the cache
            self.select_caching_algorithm(caching_algorithm)
        
        if debug_rx: self.get_logger().info(f'Appending msg with EXPIRATION TIME: {msg.duration}')
        self.cache.append(msg)
        self.event_scheduler.schedule_event(msg.duration, self.expire_callback,False,args = [msg])

        if debug_rx:
            self.get_logger().info('Message received')
            self.print_cache()

    # Different algorithms for msg removal from cache
    def remove_SPECIFIC(self,m):
        try:
            self.cache.remove(m)
            if debug_rx :self.get_logger().info(f'Removing: {m.sensor_id}-{m.sqn}')
            
        except ValueError:
            if debug_rx :self.get_logger().info(f'ERR_FIFO: {m.sensor_id}-{m.sqn}')
            pass       

    def remove_FIFO(self):
        temp_msg=self.cache[0]
        try:
            self.cache.pop(0)
            if debug_rx :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if debug_rx :self.get_logger().info(f'ERR_FIFO: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass
            
    def remove_RND(self):
        temp_msg=self.cache[random.randint(0, len(self.cache) - 1)]

        try:
            self.cache.remove(temp_msg)
            if debug_rx :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if debug_rx :self.get_logger().info(f'ERR_RND: {temp_msg.sensor_id}-{temp_msg.sqn}')       
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
            if debug_rx :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if debug_rx :self.get_logger().info(f'ERR_LRU: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass

    def remove_MRU(self):
        temp_msg=None
        for m in self.cache:
            if temp_msg==None:
                temp_msg=m
            elif m.timestamp.sec>temp_msg.timestamp.sec or (m.timestamp.sec==temp_msg.timestamp.sec and m.timestamp.nanosec>temp_msg.timestamp.nanosec):
                temp_msg=m
        try:
            self.cache.remove(temp_msg)
            if debug_rx :self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            if debug_rx :self.get_logger().info(f'ERR_LRU: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass    

    def expire_callback(self,msg):
        try:
            self.cache.remove(msg)
            if debug_rx :self.get_logger().info(f'Expired: {msg.sensor_id}-{msg.sqn}')
        except ValueError:
            if debug_rx :self.get_logger().info(f'ERR_EXP: {msg.sensor_id}-{msg.sqn}')
            pass

    def print_cache(self):
        self.get_logger().info('##############################################################')
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
    
    def execute_patrol_action(self, goal : ServerGoalHandle):

        command_goal : Patrol.Goal = goal.request

        if debug_setup:self.get_logger().info(f"Action requested. Performing movement to targets:\n\t{command_goal.targets}")

        self.fly_to_altitude(MIN_ALTITUDE_TO_PERFORM_PATROL)

        goal.succeed()

        result =  Patrol.Result()
        result.result = "Movement completed"

        return result
    
    def polling_goal_callback(self, goal):

        if debug_polling: self.get_logger().info(f'SERVER - Goal received, polling sensor {goal.req.sensor_id}')

        return rclpy.action.GoalResponse.ACCEPT
    
    def cancel_polling_callback(self, goal_handle):

        if debug_polling: self.get_logger().info(f'SERVER - Request for polling cancel received')

        return rclpy.action.CancelResponse.ACCEPT
    
    def execute_polling_action(self, goal : ServerGoalHandle):

        if debug_polling: self.get_logger().info(f'SERVER - Executing goal, polling sensor {goal.request.req.sensor_id}')
        
        requested_sensor = goal.request.req.sensor_id

        sensor_data = None
        index = 0

        for d in self.cache:
            if d.sensor_id == requested_sensor:
                if sensor_data == None:
                    sensor_data = d
                    index = self.cache.index(d)
                    
                elif d.timestamp.sec > sensor_data.timestamp.sec or (d.timestamp.sec == sensor_data.timestamp.sec and d.timestamp.nanosec > sensor_data.timestamp.nanosec):
                    sensor_data = d
                    index = self.cache.index(d)

        
        if sensor_data:
            if debug_polling: self.get_logger().info(f'SERVER - Data found: {self.cache[index].sensor_id}-{self.cache[index].sqn} (Polling sqn number {goal.request.req.sqn})')
            self.cache[index].timestamp = self.get_clock().now().to_msg()
        else:
            if debug_polling: self.get_logger().info(f'SERVER - Data NOT found for sensor {goal.request.req.sensor_id} (Polling sqn number {goal.request.req.sqn})')
        
        goal.succeed()
        
        result = Polling.Result()
        if sensor_data:
            result.result.data = sensor_data
            result.result.sqn = goal.request.req.sqn
        else:
            result.result.data.timestamp = self.get_clock().now().to_msg()
            result.result.data.duration = 0
            result.result.data.sensor_id = requested_sensor
            result.result.data.sqn = -1
            result.result.data.data = "404"
            result.result.sqn = goal.request.req.sqn

        return result

    def fly_to_altitude(self, altitude):

        # Instantiate the move_up message
        move_up = Twist()
        move_up.linear = Vector3(x=0.0, y=0.0, z=1.0)
        move_up.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.cmd_vel_publisher.publish(move_up)
        if debug_setup:self.get_logger().info(f'{altitude}')
        # Wait for the drone to reach the desired altitude
        while(self.position.z < altitude):
            time.sleep(0.1)

        # Stop movement after the altitude has been reached
        stop_mov = Twist()
        stop_mov.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_mov.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_mov)
    
    # Utility function to choose the caching algorithm
    def select_caching_algorithm(self, algorithm_id):
        if algorithm_id == 0:
            self.remove_FIFO()
        elif algorithm_id == 1:
            self.remove_RND()
        elif algorithm_id == 2:
            self.remove_LRU()
        elif algorithm_id == 3:
            self.remove_MRU()




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
