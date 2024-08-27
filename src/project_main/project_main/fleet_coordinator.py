import sys
from time import sleep
from threading import Thread
from enum import Enum
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from project_interfaces.action import Patrol
from project_main.math_utils import random_point_in_circle, is_segment_in_circles

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

        self.balloon_action_clients = {}
        self.sensor_action_clients = {}
        self.balloon_positions = {}
        self.sensor_positions = {}      
        self.balloon_states = {}
        self.sensor_states = {} 
        self.circles =[]
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
        '''
        if len(self.circles)<NUMBER_OF_BALLOONS:
            for i in range(NUMBER_OF_BALLOONS):
                var = self.balloon_positions[i]
                self.get_logger().info(f'{var}')
                self.circles.append((var.x,var.y,19))
        '''
        balloon_id=random.randint(0,NUMBER_OF_BALLOONS-1)
        chosen_balloon=self.balloon_positions[balloon_id]
        temp=random_point_in_circle((chosen_balloon.x,chosen_balloon.y),10) 
        #    if is_segment_in_circles(segment=((temp[0],temp[1]),(self.sensor_positions[sensor_id].x,self.sensor_positions[sensor_id].y)),circles=self.circles):break
        #    else: self.get_logger().info('porcodio')
         
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
        self.patrol_targets()

   

    def store_sensor_position(self, id, msg : Odometry):
        #self.get_logger().info(f"eccomi{id}")
        self.sensor_positions[id] = msg.pose.pose.position
    def store_balloon_position(self, id, msg : Odometry):
        self.balloon_positions[id] = msg.pose.pose.position


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
    fleet_coordinator.setup_balloon()
    fleet_coordinator.patrol_targets()

    executor.spin()

    executor.shutdown()
    fleet_coordinator.destroy_node()
    rclpy.shutdown()

    