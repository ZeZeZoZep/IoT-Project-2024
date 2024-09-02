import sys
import os
from dotenv import load_dotenv
from time import sleep
from threading import Thread
from random import randint
#import requests
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rosgraph_msgs.msg import Clock
from project_interfaces.action import Polling
from sim_utils import EventScheduler

load_dotenv()

WORLD_NAME = "iot_project_world"
NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

debug_polling = int(os.getenv("DEBUG_POLLING"))
is_random = int(os.getenv('IS_RANDOM'))


class BaseStationController(Node):
    
    def __init__(self):

        super().__init__('base_station_controller')

        self.polling_action_clients = {}
        self.polling_msgs_sqn = [0] * NUMBER_OF_BALLOONS
        self.seq = NUMBER_OF_SENSORS

        self.data_id = 0

        #Vars specific for cancelling goals no more needed
        self.polling_goal_handles = {}
        self.received_polling_data = False

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        for i in range(NUMBER_OF_BALLOONS):
            
            #Create an Action client for each balloon
            polling_callback_group = MutuallyExclusiveCallbackGroup()
            self.polling_action_clients[i] = ActionClient(
                self,
                Polling,
                f'/Balloon_{i}/polling',
                callback_group = polling_callback_group
            )
            self.polling_goal_handles[i] = None 
        
        #Schedule the first call of the polling action with sensor 0 and a random polling rate
        sensor_pick = self.pick_sensor()
        rate_pick = self.pick_polling_rate(is_random)

        self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [sensor_pick, rate_pick])
    
    
    #Start of the polling action
    def send_polling_requests(self, sensor_id : int, polling_rate : int):

        def send_polling_requests_inner():

            for i in range(NUMBER_OF_BALLOONS):
                self.send_polling_goal(sensor_id, polling_rate, i)

        #The separated thread will manage the execution of the functions for each balloon   
        Thread(target=send_polling_requests_inner).start()
    
    #Prepare the goal msg and send it to the Action server
    def send_polling_goal(self, sensor_id : int, polling_rate : int, uav_id : int):

        self.received_polling_data = False
        self.responses = []

        while not self.polling_action_clients[uav_id].wait_for_server(1):
            if debug_polling: self.get_logger().info("Waiting for polling action server to come online")
            sleep(3)

        polling_goal_msg = Polling.Goal()
        polling_goal_msg.req.sensor_id = sensor_id
        polling_goal_msg.req.sqn = self.generate_polling_msgs(uav_id)

        if debug_polling: self.get_logger().info(f'CLIENT - Sending Polling Goal to balloon {uav_id} for sensor {polling_goal_msg.req.sensor_id} with sqn number {polling_goal_msg.req.sqn}')
        
        
        polling_future = self.polling_action_clients[uav_id].send_goal_async(polling_goal_msg)
        polling_future.add_done_callback(lambda future, polling_rate = polling_rate : self.polling_submitted_callback(future, polling_rate, uav_id))

    #Check if the goal is accepted and add a callback to manage the actual result
    def polling_submitted_callback(self, future, polling_rate, uav_id):
        polling_goal_handle = future.result()

        if not polling_goal_handle.accepted:
            if debug_polling: self.get_logger().info('Goal rejected :(')
            return
        
        self.polling_goal_handles[uav_id] = polling_goal_handle
        
        polling_result_future = polling_goal_handle.get_result_async()
        polling_result_future.add_done_callback(lambda future, polling_rate = polling_rate : self.polling_result_callback(future, polling_rate, uav_id))
    
    #Do what you have with the result of the action
    def polling_result_callback(self, future, polling_rate, uav_id):

        result = future.result().result.result

        data_to_offload = {
            "id" : self.data_id,
            "balloon_id": uav_id,
            "data": self.ros_message_to_dict(result.data),
            "polling_sqn": result.sqn,
            "msg_receive_timestamp": self.time_to_dict(self.get_clock().now().to_msg())
        }
        self.offload_data_to_file(data_to_offload)

        polling_rate -= 1
        if result.data.data == "404":
            self.responses.append(result)
            if debug_polling: self.get_logger().info(f'CLIENT - Balloon {uav_id}. No data found for sensor {result.data.sensor_id}, polling req sqn number {result.sqn}')
            if len(self.responses) == NUMBER_OF_BALLOONS:
                if polling_rate == 0:
                    sensor_pick = self.pick_sensor()
                    rate_pick = self.pick_polling_rate(is_random)
                    self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [sensor_pick, rate_pick])
                else:
                    if debug_polling: self.get_logger().info(f'CLIENT - Remaining polling request: {polling_rate}')
                    self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [result.data.sensor_id , polling_rate])

        else:

            if not self.received_polling_data:
                self.received_polling_data = True
                self.cancel_remaining_polling_goals(uav_id)
            
            if debug_polling: self.get_logger().info(f'CLIENT - Balloon {uav_id}. Result data for sensor {result.data.sensor_id}, sensor sqn number {result.data.sqn}: {result.data.data}')

            if polling_rate == 0:
                sensor_pick = self.pick_sensor()
                rate_pick = self.pick_polling_rate(is_random)
                self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [sensor_pick, rate_pick])
            else:
                if debug_polling: self.get_logger().info(f'CLIENT - Remaining polling request: {polling_rate}')
                self.event_scheduler.schedule_event(1, self.send_polling_requests, False, args = [result.data.sensor_id, polling_rate])
    
    def cancel_remaining_polling_goals(self, completed_uav_id):
        if debug_polling: self.get_logger().info(f'CLIENT - Canceling pending goals of other ballons except balloon {completed_uav_id}')

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

    #OFFLOAD SU SERVER
    """ def offload_data_to_server(self, data):
        server_url = "http://example.com/offload"  # Inserisci l'URL del tuo server
        try:
            response = requests.post(server_url, json=data)
            if response.status_code == 200:
                if debug_polling: self.get_logger().info("Data offloading successful")
            else:
                if debug_polling: self.get_logger().info(f"Data offloading failed with status code {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error during data offloading: {e}")  """        

    #OFFLOAD SU FILE 
    def offload_data_to_file(self, data):
        
        filename = "offloaded_data.json"  # Nome del file dove verranno salvati i dati
        try:
            with open(filename, "a+", encoding='utf-8') as file:
                file.write(json.dumps(data, ensure_ascii='utf-8', indent=4) + ",\n")

            self.data_id += 1
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
    
    #Utility functions for sensor pick, polling rate and sqn number generator for the base station's requests
    def pick_sensor(self):
        return randint(0, NUMBER_OF_SENSORS - 1)

    def pick_polling_rate(self, flag):
        if flag:
            return 1
        else:
            return randint(2, 5)
    
    def generate_polling_msgs(self, i):
        self.polling_msgs_sqn[i] += 1
        return self.polling_msgs_sqn[i]
    

def main():

    rclpy.init()
    executor = MultiThreadedExecutor()
    base_station_controller = BaseStationController()

    executor.add_node(base_station_controller)

    executor.spin()

    base_station_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    