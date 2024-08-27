import sys
from time import sleep
from threading import Thread
from random import randint

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from project_interfaces.action import Polling
from project_interfaces.msg import Req

from sim_utils import EventScheduler

WORLD_NAME = "iot_project_world"
NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

DEBUG_POLLING = True


class BaseStationController(Node):
    
    def __init__(self):

        super().__init__('base_station_controller')

        self.polling_action_clients = {}
        self.polling_msgs_sqn = [0] * NUMBER_OF_BALLOONS
        self.seq = NUMBER_OF_SENSORS

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        for i in range(NUMBER_OF_BALLOONS):

            polling_callback_group = MutuallyExclusiveCallbackGroup()
            self.polling_action_clients[i] = ActionClient(
                self,
                Polling,
                f'/Balloon_{i}/polling',
                callback_group = polling_callback_group
            )    
        
        sensor_pick = 0 % NUMBER_OF_SENSORS
        random_rate = self.pick_polling_rate()

        self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [sensor_pick, random_rate])
    
    def pick_polling_rate(self):
        return randint(2, 5)
    
    def generate_polling_msgs(self, i):
        self.polling_msgs_sqn[i] += 1
        return self.polling_msgs_sqn[i]
    
    def send_polling_requests(self, sensor_id : int, polling_rate : int):

        def send_polling_requests_inner():

            for i in range(NUMBER_OF_BALLOONS):
                self.send_polling_goal(sensor_id, polling_rate, i)
            
        Thread(target=send_polling_requests_inner).start()
    
    def send_polling_goal(self, sensor_id : int, polling_rate : int, uav_id : int):

        while not self.polling_action_clients[uav_id].wait_for_server(1):
            if DEBUG_POLLING: self.get_logger().info("Waiting for polling action server to come online")
            sleep(3)
        
        self.responses = []
        polling_goal_msg = Polling.Goal()
        polling_goal_msg.req.sensor_id = sensor_id
        polling_goal_msg.req.sqn = self.generate_polling_msgs(uav_id)
        #self.get_logger().info(f'Sqn number for polling request forward ballon {uav_id}: {self.polling_msgs_sqn[uav_id]}')

        if DEBUG_POLLING: self.get_logger().info(f'Sending Polling Goal to balloon {uav_id} for sensor {polling_goal_msg.req.sensor_id} with sqn number {polling_goal_msg.req.sqn}')
        
        
        polling_future = self.polling_action_clients[uav_id].send_goal_async(polling_goal_msg)
        polling_future.add_done_callback(lambda future, polling_rate = polling_rate : self.polling_submitted_callback(future, polling_rate))

    def polling_submitted_callback(self, future, polling_rate):
        polling_goal_handle = future.result()

        if not polling_goal_handle.accepted:
            if DEBUG_POLLING: self.get_logger().info('Goal rejected :(')
            return
        
        polling_result_future = polling_goal_handle.get_result_async()
        polling_result_future.add_done_callback(lambda future, polling_rate = polling_rate : self.polling_result_callback(future, polling_rate))
    
    def polling_result_callback(self, future, polling_rate):
        result = future.result().result
        polling_rate -= 1
        if result.result.data == "404":
            self.responses.append(result.result)
            if DEBUG_POLLING: self.get_logger().info(f'No data found for sensor {result.result.sensor_id}, polling sqn number {result.result.sqn}')
            if len(self.responses) == NUMBER_OF_BALLOONS:
                if polling_rate == 0:
                    sensor_pick = (result.result.sensor_id + 1) % NUMBER_OF_SENSORS
                    random_rate = self.pick_polling_rate()
                    self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [sensor_pick, random_rate])
                else:
                    if DEBUG_POLLING: self.get_logger().info(f'Remaining polling request: {polling_rate}')
                    self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [result.result.sensor_id , polling_rate])
        else:
            self.responses.append(result.result)
            if DEBUG_POLLING: self.get_logger().info(f'Result data for sensor {result.result.sensor_id}, sensor sqn number {result.result.sqn}: {result.result.data}')
            if len(self.responses) == NUMBER_OF_BALLOONS:
                if polling_rate == 0:
                    sensor_pick = (result.result.sensor_id + 1) % NUMBER_OF_SENSORS
                    random_rate = self.pick_polling_rate()
                    self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [sensor_pick, random_rate])
                else:
                    if DEBUG_POLLING: self.get_logger().info(f'Remaining polling request: {polling_rate}')
                    self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [result.result.sensor_id, polling_rate])


def main():

    rclpy.init()
    executor = MultiThreadedExecutor()
    base_station_controller = BaseStationController()

    executor.add_node(base_station_controller)

    executor.spin()

    base_station_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    