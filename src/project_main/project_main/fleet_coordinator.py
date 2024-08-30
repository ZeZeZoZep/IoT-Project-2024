import sys
from time import sleep
from threading import Thread
from enum import Enum
import random
from random import randint

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
#from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from project_interfaces.action import Patrol
from project_main.math_utils import random_point_in_circle, is_segment_in_circles
#from project_interfaces.action import Polling

from sim_utils import EventScheduler

WORLD_NAME = "iot_project_world"
NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

HOVERING_HEIGHT = 5.0

DEBUG_SETUP= False
DEBUG_PATROLLING= False
class FleetCoordinator(Node):

    """
    Fleet Coordinator class, used the manage the whole fleet of Balloons and Drones.
    This is where most of the task should be submitted by default.
    """
    
    def __init__(self):

        super().__init__('fleet_coordinator')

        #self.polling_action_clients = {}
        self.balloon_action_clients = {}
        self.sensor_action_clients = {}
        self.balloon_positions = {}
        self.sensor_positions = {}      
        self.sensor_states = {} 
        self.circles =[]
        self.balloon_states = {} 

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        for i in range(NUMBER_OF_BALLOONS):

            self.balloon_action_clients[i] = ActionClient(
                    self,
                    Patrol,
                    f'/Balloon_{i}/patrol'
                )
            
            self.balloon_states[i] = BalloonState.LANDED
            
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda msg, id = i : self.store_balloon_position(id, msg),
                10
            )

            """ polling_callback_group = ReentrantCallbackGroup()
            self.polling_action_clients[i] = ActionClient(
                self,
                Polling,
                f'/Balloon_{i}/polling',
                callback_group = polling_callback_group
            ) """


        for i in range(NUMBER_OF_SENSORS):
            self.sensor_action_clients[i] = ActionClient(
                    self,
                    Patrol,
                    f'/ActiveSensor_{i}/patrol'
                )
            
            self.sensor_states[i] = SensorState.STILL

            self.create_subscription(
                Odometry,
                f'ActiveSensor_{i}/odometry',
                lambda msg, id = i : self.store_sensor_position(id, msg),
                10
            )
        
    
        """ random_sensor = self.pick_random_sensor()
        random_rate = self.pick_polling_rate() """

        self.event_scheduler.schedule_event(1, self.setup_balloon, False, args = [])
        #self.event_scheduler.schedule_event(1, self.send_polling_goal, False, args = [random_sensor, random_rate])
    def setup_balloon(self):

        
        #Method used to keep the fleet of Balloons constantly patrolling the set of targets.
        #When a patrolling task has been completed, a new one with the same targets is given again.
        

        def setup_balloon_inner():


            
            for i in range(NUMBER_OF_BALLOONS):
                # Do not resubmit tasks to already moving balloons
                if not self.balloon_states[i] is BalloonState.MOVING:
                    self.submit_task_balloon(i)
            
        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=setup_balloon_inner).start()
    def submit_task_balloon(self, uav_id : int):

        # Wait for the action server to go online
        while not self.balloon_action_clients[uav_id].wait_for_server(1) or len(self.sensor_positions) < NUMBER_OF_SENSORS:
            if DEBUG_SETUP:self.get_logger().info(f"{self.balloon_action_clients[uav_id].wait_for_server(1)}or{len(self.sensor_positions) < NUMBER_OF_SENSORS} Waiting for action server to come online and sensors to announce their position")
            sleep(3)

        # Set the Balloon to moving state
        self.balloon_states[uav_id] = BalloonState.MOVING
        goal = Patrol.Goal()
        goal.targets = []

        # Just iterate through all the sensors in a round-robin fashion
        goal.targets.append(self.balloon_positions[uav_id])

        # Set the height to a predefined value, as the sensors will be on the ground, we want to hover them
        goal.targets[0].z = HOVERING_HEIGHT


        if DEBUG_SETUP:self.get_logger().info(f"Submitting task for Balloon {uav_id}")

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.balloon_action_clients[uav_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, uav_id = uav_id : self.setup_balloon_submitted_callback(uav_id, future))    
    def setup_balloon_submitted_callback(self, uav_id, future):

        # Check if the patrol action was accepted
        goal_handle = future.result()
    
        if not goal_handle.accepted:
            # If not, set the balloon back to hovering, and return
            if DEBUG_SETUP:self.get_logger().info("Task has been refused by the action server")
            self.balloon_states[uav_id] = BalloonState.HOVERING
            return
        
        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(lambda future, uav_id = uav_id : self.setup_balloon_completed_callback(uav_id, future))
    def setup_balloon_completed_callback(self, uav_id, future):
        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
        # you may have to handle such cases
        if DEBUG_SETUP:self.get_logger().info(f"Patrolling action for Balloon {uav_id} has been completed. Drone is going idle")
        self.balloon_states[uav_id] = BalloonState.HOVERING
        self.event_scheduler.schedule_event(1, self.patrol_targets, False, args = [])
    
    def patrol_targets(self):

        
        #Method used to keep the fleet of Balloons constantly patrolling the set of targets.
        #When a patrolling task has been completed, a new one with the same targets is given again.
        

        def patrol_targets_inner():

            while True:
                for i in range(NUMBER_OF_SENSORS):
                    # Do not resubmit tasks to already moving balloons
                    if not self.sensor_states[i] is SensorState.MOVING:
                        self.submit_task_sensor(i)
            
        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=patrol_targets_inner).start()

    
    def submit_task_sensor(self,sensor_id : int):

        # Wait for the action server to go online
        while not self.sensor_action_clients[sensor_id].wait_for_server(1) or (len(self.sensor_positions) < NUMBER_OF_SENSORS and len(self.balloon_positions) < NUMBER_OF_BALLOONS): 
            if DEBUG_PATROLLING:self.get_logger().info("Waiting for sensors")
            sleep(3)
        temp=None
        balloon_id=random.randint(0,NUMBER_OF_BALLOONS-1)
        chosen_balloon=self.balloon_positions[balloon_id]
        temp=random_point_in_circle((chosen_balloon.x,chosen_balloon.y),10) 

         
        # Set the Balloon to moving state
        self.sensor_states[sensor_id] = SensorState.MOVING
        goal = Patrol.Goal()
        goal.targets = []

        # Just iterate through all the sensors in a round-robin fashion
        goal.targets.append(Point(x=temp[0],y=temp[1],z=0.701))
    



        self.get_logger().info(f"Submitting task for sensor{sensor_id}")

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.sensor_action_clients[sensor_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, sensor_id = sensor_id : self.patrol_submitted_callback(sensor_id, future))


    def patrol_submitted_callback(self, sensor_id, future):

        # Check if the patrol action was accepted
        goal_handle = future.result()
    
        if not goal_handle.accepted:
            # If not, set the balloon back to hovering, and return
            if DEBUG_PATROLLING:self.get_logger().info("Task has been refused by the action server")
            self.sensor_states[sensor_id] = SensorState.MOVING
            return
        
        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(lambda future, sensor_id = sensor_id: self.patrol_completed_callback(sensor_id, future))


    def patrol_completed_callback(self, sensor_id, future):

        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
        # you may have to handle such cases
        if DEBUG_PATROLLING:self.get_logger().info(f"Patrolling action for Sensor{sensor_id} has been completed. Drone is going idle")
        self.sensor_states[sensor_id] = SensorState.STILL
        self.event_scheduler.schedule_event(1, self.patrol_targets, False, args = [])

   

    def store_sensor_position(self, id, msg : Odometry):
        #self.get_logger().info(f"eccomi{id}")
        self.sensor_positions[id] = msg.pose.pose.position
    def store_balloon_position(self, id, msg : Odometry):
        self.balloon_positions[id] = msg.pose.pose.position

    """ def pick_random_sensor(self):
        return randint(0, NUMBER_OF_SENSORS - 1)
    
    def pick_polling_rate(self):
        return randint(2, 6)
    
    def send_polling_goal(self, sensor_id : int, polling_rate : int):
        self.responses = []
        polling_goal_msg = Polling.Goal()
        polling_goal_msg.sensor_id = sensor_id

        for i in range(NUMBER_OF_BALLOONS):

            while not self.polling_action_clients[i].wait_for_server(1):
                self.get_logger().info("Waiting for polling action server to come online")
                sleep(3)
            
            self.polling_future = self.polling_action_clients[i].send_goal_async(polling_goal_msg)
            self.polling_future.add_done_callback(lambda future, polling_rate = polling_rate : self.polling_submitted_callback(future, polling_rate))
    
    def polling_submitted_callback(self, future, polling_rate):
        polling_goal_handle = future.result()

        if not polling_goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.polling_result_future = polling_goal_handle.get_result_async()
        self.polling_result_future.add_done_callback(lambda future, polling_rate = polling_rate : self.polling_result_callback(future, polling_rate))
    
    def polling_result_callback(self, future, polling_rate):
        result = future.result().result
        if result.result.data == "404":
            self.responses.append(result.result)
            self.get_logger().info(f'{result.result.data}')
            if len(self.responses) == NUMBER_OF_BALLOONS:
                if polling_rate == 0:
                    random_sensor = self.pick_random_sensor()
                    random_rate = self.pick_polling_rate()
                    self.event_scheduler.schedule_event(1, self.send_polling_goal, False, args = [random_sensor, random_rate])
                else:
                    self.get_logger().info(f'Polling rate: {polling_rate}')
                    polling_rate -= 1
                    self.event_scheduler.schedule_event(1, self.send_polling_goal, False, args = [result.result.sensor_id, polling_rate])
        else:
            self.responses.append(result.result)
            self.get_logger().info(f'{result.result.data}')
            if polling_rate == 0:
                random_sensor = self.pick_random_sensor()
                random_rate = self.pick_polling_rate()
                self.event_scheduler.schedule_event(1, self.send_polling_goal, False, args = [random_sensor, random_rate])
            else:
                self.get_logger().info(f'Polling rate: {polling_rate}')
                polling_rate -= 1
                self.event_scheduler.schedule_event(1, self.send_polling_goal, False, args = [result.result.sensor_id, polling_rate]) """


class BalloonState(Enum):
    LANDED = 1
    HOVERING = 2
    MOVING = 3

class SensorState(Enum):
    STILL = 1
    MOVING = 2


def main():

    rclpy.init()
    executor = MultiThreadedExecutor()
    fleet_coordinator = FleetCoordinator()

    executor.add_node(fleet_coordinator)

    executor.spin()

    fleet_coordinator.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

    