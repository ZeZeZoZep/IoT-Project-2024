import os
from dotenv import load_dotenv
from time import sleep
import threading
from threading import Thread
from random import randint, random
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from project_interfaces.action import Polling
from sim_utils import EventScheduler

load_dotenv()
lock = threading.Lock()

WORLD_NAME = "iot_project_world"

number_of_balloons = int(os.getenv('NUMBER_OF_BALLOONS'))
number_of_sensors = int(os.getenv('NUMBER_OF_SENSORS'))
debug_polling = int(os.getenv("DEBUG_POLLING"))


class BaseStationController(Node):
    
    def __init__(self):

        super().__init__('base_station_controller')

        self.poissonian_interrarival_rates = [0.0] * number_of_sensors
        self.polling_action_clients = {}
        self.polling_msgs_sqn = 0
        file_path = "offloaded_data.json"

        if os.path.exists(file_path):
            os.remove(file_path)

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        for i in range(number_of_balloons):
            
            # Create an Action client for each balloon
            polling_callback_group = MutuallyExclusiveCallbackGroup()
            self.polling_action_clients[i] = ActionClient(
                self,
                Polling,
                f'/Balloon_{i}/polling',
                callback_group = polling_callback_group
            )

        for i in range(number_of_sensors):
            self.poissonian_interrarival_rates[i] = self.pick_poissonian_interrarival_rate()
        
        self.event_scheduler.schedule_event(1, self.call_polling_task, False, args = [])
    
    
    def call_polling_task(self):

        for i in range(number_of_sensors):
            Thread(target=self.send_polling_requests, args=[i]).start()

    # Start of the polling action
    def send_polling_requests(self, sensor_id : int):

        thread_specific_interrarrival_rate = self.poissonian_interrarival_rates[sensor_id]

        # The separated thread will manage the execution of the functions for each balloon
        Thread(target=self.send_polling_requests_inner, args=[sensor_id]).start()
        
        self.event_scheduler.schedule_event(np.random.exponential(1 / thread_specific_interrarrival_rate), self.send_polling_requests, False, args = [sensor_id])
    
    def send_polling_requests_inner(self, sensor_id):

        self.responses = []
        with lock:
            polling_sqn = self.generate_polling_msgs()

        for i in range(number_of_balloons):
            self.send_polling_goal(sensor_id, i, polling_sqn)

    # Prepare the goal msg and send it to the Action server
    def send_polling_goal(self, sensor_id : int, uav_id : int, polling_sqn : int):

        # Vars specific for cancelling goals no more needed
        self.polling_goal_handles = {}
        self.received_polling_data = False

        while not self.polling_action_clients[uav_id].wait_for_server(1):
            if debug_polling: self.get_logger().info("Waiting for polling action server to come online")
            sleep(3)

        polling_goal_msg = Polling.Goal()
        polling_goal_msg.req.sensor_id = sensor_id
        polling_goal_msg.req.sqn = polling_sqn

        if debug_polling: self.get_logger().info(f'CLIENT - Sending Polling Goal to balloon {uav_id} for sensor {polling_goal_msg.req.sensor_id} with sqn number {polling_goal_msg.req.sqn}')
        
        
        polling_future = self.polling_action_clients[uav_id].send_goal_async(polling_goal_msg)
        polling_future.add_done_callback(lambda future, uav_id = uav_id : self.polling_submitted_callback(future, uav_id))

    # Check if the goal is accepted and add a callback to manage the actual result
    def polling_submitted_callback(self, future, uav_id):
        polling_goal_handle = future.result()

        if not polling_goal_handle.accepted:
            if debug_polling: self.get_logger().info('Goal rejected :(')
            return
        
        with lock:
            self.polling_goal_handles[uav_id] = polling_goal_handle
        
        polling_result_future = polling_goal_handle.get_result_async()
        polling_result_future.add_done_callback(lambda future, uav_id = uav_id : self.polling_result_callback(future, uav_id))
    
    # Do what you have with the result of the action
    def polling_result_callback(self, future, uav_id):

        result = future.result().result.result

        if result.data.data == "404":
            if debug_polling: self.get_logger().info(f'CLIENT - Balloon {uav_id}. No data found for sensor {result.data.sensor_id}, polling req sqn number {result.sqn}')
            self.responses.append(result)

        else:
            if debug_polling: self.get_logger().info(f'CLIENT - Balloon {uav_id}. Result data for sensor {result.data.sensor_id}, sensor sqn number {result.data.sqn}: {result.data.data}')

            if not self.received_polling_data:
                self.received_polling_data = True
                self.cancel_remaining_polling_goals(uav_id)
        
        data_to_offload = {
            "balloon_id": uav_id,
            "data": self.ros_message_to_dict(result.data),
            "polling_sqn": result.sqn,
            "msg_receive_timestamp": self.time_to_dict(self.get_clock().now().to_msg())
        }

        self.offload_data_to_file(data_to_offload)
    
    def cancel_remaining_polling_goals(self, completed_uav_id):
        if debug_polling: self.get_logger().info(f'CLIENT - Canceling pending goals of other ballons except balloon {completed_uav_id}')
        with lock:
            for uav_id, goal_handle in self.polling_goal_handles.items():
                if goal_handle is not None and uav_id != completed_uav_id:
                    cancel_polling_future = goal_handle.cancel_goal_async()
                    cancel_polling_future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        cancel_response = future.result()

        if cancel_response.goals_canceling:
            if debug_polling: self.get_logger().info('CLIENT - Goal successfully cancelled')
            else:
                self.get_logger().info('CLIENT - Impossible to cancel goal')     

    # Offload to file
    def offload_data_to_file(self, data):
        filename = "offloaded_data.json"  # Name of file where data will be offloaded

        with lock:
            try:
                with open(filename, "a+", encoding='utf-8') as file:
                    file.write(json.dumps(data, ensure_ascii='utf-8', indent=4) + ",\n")

                if debug_polling: self.get_logger().info("Data successfully offloaded to file")

            except IOError as e:
                self.get_logger().error(f"Error during file offloading: {e}")     

    # Utility functions for data serialization
    def ros_message_to_dict(self, msg):

        # Using the built-in method to convert ROS 2 message to a dictionary
        if hasattr(msg, '__slots__') and hasattr(msg, '_fields_and_field_types'):

            msg_dict = {}
            for field_name in msg.__slots__:
                field_value = getattr(msg, field_name)

                if field_name == "_timestamp":
                    field_value = self.time_to_dict(field_value)
                
                msg_dict[field_name] = field_value
            return msg_dict
        else:
            raise ValueError("Input is not a valid ROS message")    
        
    # Convert Time msg in a dict obj
    def time_to_dict(self, time_msg):

        return {
            'sec': time_msg.sec,
            'nanosec': time_msg.nanosec
        }
    
    # Utility functions for sensor pick, polling rate and sqn number generator for the base station's requests
    def pick_sensor(self):
        return randint(0, number_of_sensors - 1)
    
    def generate_polling_msgs(self):
        self.polling_msgs_sqn += 1
        return self.polling_msgs_sqn
    
    # Pick a Poissonian rate between 0,2 and 1
    def pick_poissonian_interrarival_rate(self):
        return (random() * 0.8) + 0.2
    

def main():

    rclpy.init()
    executor = MultiThreadedExecutor()
    base_station_controller = BaseStationController()

    executor.add_node(base_station_controller)

    executor.spin()

    base_station_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    