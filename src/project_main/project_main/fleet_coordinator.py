import sys
import os
import ast
import random
import math
import rclpy

from dotenv import load_dotenv
from time import sleep
from threading import Thread
from enum import Enum

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from project_interfaces.action import Patrol
from project_main.math_utils import random_point_in_circle, polar_to_euclidian
import networkx as nx

from sim_utils import EventScheduler

load_dotenv()

WORLD_NAME = "iot_project_world"
HOVERING_HEIGHT = 5.0

debug_setup = int(os.getenv('DEBUG_SETUP'))
debug_patrolling = int(os.getenv('DEBUG_PATROLLING'))
number_of_balloons = int(os.getenv('NUMBER_OF_BALLOONS'))
number_of_sensors = int(os.getenv('NUMBER_OF_SENSORS'))
number_of_lidar_sensors = int(os.getenv('NUMBER_OF_LIDAR_SENSORS'))


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
        self.sensor_states = {} 
        self.circles =[]
        self.balloon_states = {}
        self.G = nx.Graph()
        self.points = {}
        self.edges = []
        self.markings=ast.literal_eval(sys.argv[1])

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        for i in range(number_of_balloons):

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

        for i in range(number_of_sensors):
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
        
        self.setup_graph()
        self.event_scheduler.schedule_event(1, self.setup_balloon, False, args = [])
        self.event_scheduler.schedule_event(5, self.patrol_targets, False, args = [])

    def setup_balloon(self):
        
        #Method used to keep the fleet of Balloons constantly patrolling the set of targets.

        def setup_balloon_inner():
            
            for i in range(number_of_balloons):
                # Do not resubmit tasks to already moving balloons
                if not self.balloon_states[i] is BalloonState.MOVING:
                    self.submit_task_balloon(i)
            
        # Start this function in another thread, so that the node can start spinning immediately after this function has finished
        Thread(target=setup_balloon_inner).start()
    
    def submit_task_balloon(self, uav_id : int):
        
        # Wait for the action server to go online
        self.wait_for_balloons(uav_id)
        # Set the Balloon to moving state
        self.balloon_states[uav_id] = BalloonState.MOVING
        goal = Patrol.Goal()
        goal.targets = []

        # Just iterate through all the sensors in a round-robin fashion
        goal.targets.append(self.balloon_positions[uav_id])

        # Set the height to a predefined value, as the sensors will be on the ground, we want to hover them
        goal.targets[0].z = HOVERING_HEIGHT


        if debug_setup:self.get_logger().info(f"Submitting task for Balloon {uav_id}")

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.balloon_action_clients[uav_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, uav_id = uav_id : self.setup_balloon_submitted_callback(uav_id, future))    
    def setup_balloon_submitted_callback(self, uav_id, future):

        # Check if the patrol action was accepted
        goal_handle = future.result()
    
        if not goal_handle.accepted:
            # If not, set the balloon back to hovering, and return
            if debug_setup:self.get_logger().info("Task has been refused by the action server")
            self.balloon_states[uav_id] = BalloonState.HOVERING
            return
        
        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(lambda future, uav_id = uav_id : self.setup_balloon_completed_callback(uav_id, future))
    def setup_balloon_completed_callback(self, uav_id, future):
        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
        # you may have to handle such cases
        if debug_setup:self.get_logger().info(f"Patrolling action for Balloon {uav_id} has been completed. Drone is going idle")
        self.balloon_states[uav_id] = BalloonState.HOVERING
        
    
    def patrol_targets(self):


        #Method used to keep the fleet of Balloons constantly patrolling the set of targets.
        #When a patrolling task has been completed, a new one with the same targets is given again.
        def patrol_targets_inner():
            for i in range(number_of_sensors):
                if i<number_of_lidar_sensors:
                    # Do not resubmit tasks to already moving balloons
                    if not self.sensor_states[i] is SensorState.MOVING:
                        self.submit_task_lidar_sensor(i)
                else:
                    if not self.sensor_states[i] is SensorState.MOVING:
                        self.submit_task_sensor(i)


            
        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=patrol_targets_inner).start()

    
    def submit_task_sensor(self,sensor_id : int):
        self.get_logger().info(f'Sensor: {sensor_id}, Lidar: NO')
        self.wait_for_sensors(sensor_id)

        
        goal = Patrol.Goal()
        goal.targets = []

        #self.get_logger().info(f'{self.markings}')  
        index_occupied_node=self.markings.get(sensor_id)
        neighbours=self.G[index_occupied_node]
        #self.get_logger().info(f'{neighbours}') 
        aux_list=list(set(neighbours.keys()).difference(set(self.markings.values())))
        if(len(aux_list)>0):
            #for elem in aux_list:
            #    self.get_logger().info(f'{elem}')    
            #self.get_logger().info(f'lista{aux_list}')
            #self.get_logger().info(f'cacca{len(aux_list)}cacca')
            random_neighbour=random.randint(0,len(aux_list)-1)
            #self.get_logger().info(f'ciao{random_neighbour}')
            target=aux_list[random_neighbour]
            #self.get_logger().info(f'ciao{target}')
            #self.get_logger().info(f'ciaopaloneg{self.points[target]}')  
            goal.targets.append(self.points[target])
            self.markings.update({sensor_id:target})
            #self.get_logger().info(f'lamerda {sensor_id}:{target}')
            self.sensor_states[sensor_id] = SensorState.MOVING
         
        # Set the Balloon to moving state




        if debug_patrolling:self.get_logger().info(f"Submitting task for sensor{sensor_id}")

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.sensor_action_clients[sensor_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, sensor_id = sensor_id : self.patrol_submitted_callback(sensor_id, future))
    
    def submit_task_lidar_sensor(self,sensor_id : int):
        self.get_logger().info(f'Sensor: {sensor_id}, Lidar: YES')

        # Wait for the action server to go online
        self.wait_for_sensors(sensor_id)

        balloon_id=random.randint(0,number_of_balloons-1)
        chosen_balloon=self.balloon_positions[balloon_id]
        temp=random_point_in_circle((chosen_balloon.x,chosen_balloon.y),10) 

         
        # Set the Balloon to moving state
        self.sensor_states[sensor_id] = SensorState.MOVING
        goal = Patrol.Goal()
        goal.targets = []

        # Just iterate through all the sensors in a round-robin fashion
        goal.targets.append(Point(x=temp[0],y=temp[1],z=0.701))
    



        if debug_patrolling:self.get_logger().info(f"Submitting task for sensor{sensor_id}")

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.sensor_action_clients[sensor_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, sensor_id = sensor_id : self.patrol_submitted_callback(sensor_id, future))



    def patrol_submitted_callback(self, sensor_id, future):

        # Check if the patrol action was accepted
        goal_handle = future.result()
    
        if not goal_handle.accepted:
            # If not, set the balloon back to hovering, and return
            if debug_patrolling:self.get_logger().info("Task has been refused by the action server")
            self.sensor_states[sensor_id] = SensorState.MOVING
            return
        
        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(lambda future, sensor_id = sensor_id: self.patrol_completed_callback(sensor_id, future))


    def patrol_completed_callback(self, sensor_id, future):

        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
        # you may have to handle such cases
        if debug_patrolling:self.get_logger().info(f"Patrolling action for Sensor{sensor_id} has been completed. Drone is going idle")
        self.sensor_states[sensor_id] = SensorState.STILL
        if sensor_id<number_of_lidar_sensors:self.event_scheduler.schedule_event(1, self.submit_task_lidar_sensor, False, args = [sensor_id])
        else: self.event_scheduler.schedule_event(1, self.submit_task_sensor, False, args = [sensor_id])

   

    def store_sensor_position(self, id, msg : Odometry):
        #self.get_logger().info(f"eccomi{id}")
        self.sensor_positions[id] = msg.pose.pose.position
    def store_balloon_position(self, id, msg : Odometry):
        self.balloon_positions[id] = msg.pose.pose.position
    def setup_graph(self):
        balloon_spawn_positions=[]

        #CALCOLA SPAWNING POSITIONS DEI BALLOON: caso <=3 balloon
        if number_of_balloons<4:
            for i in range(number_of_balloons):
                punto=tuple()
                if i==0:
                    punto=(0.0, 0.0, 0.0)
                elif i==1:
                    punto = (+32.0, 0.0, 0.0)
                elif i==2:
                    punto = (+16.0,-27.71, 0.0)
                balloon_spawn_positions.append(self.tuple_to_point(punto))
        
        #CALCOLA SPAWNING POSITIONS DEI BALLOON: caso >3 balloon
        else:
            for i in range(number_of_balloons):
                punto=tuple()
                if i%3==0:
                    punto=((i/3)*32.0, 0.0, 0.0)
                elif i%3==1:
                    punto=((((i-1)/3)*32.0)+16.0, 27.71, 0.0)
                else:
                    punto=((((i-2)/3)*32.0)+16.0, -27.71, 0.0)
                balloon_spawn_positions.append(self.tuple_to_point(punto))

        for index in range(number_of_balloons):
            b_position=balloon_spawn_positions[index]
            b_position.z=0.701
            self.create_node(index*7,b_position)

            #CREA GRAFO ESAGONO
            for i in range(6):
                new_point=polar_to_euclidian(12,(i*math.pi/3),b_position)
                self.create_node(index*7+(i+1),new_point)
                self.create_link(index*7,index*7+(i+1))
                if i != 0: self.create_link(index*7+(i+1),index*7+i)
                if i == 5: 
                    self.create_link(index*7+(i+1),index*7+1)
                    #self.get_logger().info(f'nuovo edge {index*7+(i+1)} - {index*7+1}')

            #CONNETTI COMPONENTI ESAGONALI: caso <= 3 balloon
            if number_of_balloons<4:
                if index==0:pass
                elif index==1:

                    index2=index-1
                    self.create_link(index*7+4,index2*7+1)

                elif index==2:

                    index2=index-2
                    self.create_link(index*7+3,index2*7+6)
                    index2=index-1
                    self.create_link(index*7+5,index2*7+2)
            #CONNETTI COMPONENTI ESAGONALI: caso > 3 balloon
            else:
                if i%3==0:
                    if index-3>0: 
                        index2=index-3
                        self.create_link(index*7+4,index2*7+1)
                    if index-2>0: 
                        index2=index-2
                        self.create_link(index*7+3,index2*7+6)
                    if index-1>0: 
                        index2=index-1
                        self.create_link(index*7+5,index2*7+2)
                elif i%3==1:
                    if index-3>0: 
                        index2=index-3
                        self.create_link(index*7+4,index2*7+1)
                    if index-1>0: 
                        index2=index-1
                        self.create_link(index*7+5,index2*7+2)

                else:
                    if index-3>0: 
                        index2=index-3
                        self.create_link(index*7+4,index2*7+1)
                    if index-2>0: 
                        index2=index-2
                        self.create_link(index*7+3,index2*7+6)
            #self.get_logger().info(f'{self.points.keys()}')
            #self.get_logger().info(f'{self.edges}')
        for point, coord in self.points.items():
            self.G.add_node(point, pos=coord)
            #self.get_logger().info(f'{point}')

    # Aggiungi lati (come tuple dei punti connessi)
        self.G.add_edges_from(self.edges)

    def wait_for_balloons(self,uav_id):
        flag=not self.balloon_action_clients[uav_id].wait_for_server(1)
        if debug_setup:self.get_logger().info(f"ORCODIOOOOOO: {flag}")
        while flag:
            sleep(3)
            flag=not self.balloon_action_clients[uav_id].wait_for_server(1)
            if debug_setup:self.get_logger().info(f"ORCODIOOOOOO: {flag}")
        if debug_setup:self.get_logger().info(f"Controllo se so dove sono")   

        while len(self.balloon_positions) !=number_of_balloons :
            if debug_setup:self.get_logger().info(f"dove sooonooooo?") 
            sleep(3)

        if debug_setup:self.get_logger().info(f"ECCOCI =)")  

    def wait_for_sensors(self,sensor_id):
        flag=not self.sensor_action_clients[sensor_id].wait_for_server(1)
        if debug_setup:self.get_logger().info(f"ORCODIOOOOOO: {flag}")
        while flag:
            sleep(3)
            flag=not self.sensor_action_clients[sensor_id].wait_for_server(1)
            if debug_setup:self.get_logger().info(f"ORCODIOOOOOO: {flag}")
        if debug_setup:self.get_logger().info(f"Controllo se so dove sono i plaons")   

        while len(self.balloon_positions) < number_of_balloons :
            if debug_setup:self.get_logger().info(f"dove sono i miei amici?") 
            sleep(3)

        if debug_setup:self.get_logger().info(f"ECCOCI =)")  


    def point_to_tuple(self,point):
        return (point.x,point.y,point.z)
    
    def tuple_to_point(self,object):
        ret=Point()
        ret.x=object[0]
        ret.y=object[1]
        ret.z=object[2]
        return ret
    
    def create_node(self,integer,point):
        self.points.update({integer:point})
        
    def create_link(self,point1,point2):
        self.edges.append((point1,point2))

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

    