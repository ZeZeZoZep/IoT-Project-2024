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

from sim_utils import EventScheduler

WORLD_NAME = "iot_project_world"
MIN_ALTITUDE_TO_PERFORM_PATROL = 15
SIZE = 10


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


    def rx_callback(self, msg : Data):

        #print the cache length
        #self.get_logger().info(f'{len(self.cache)}')
        
        if len(self.cache)>=self.cache_size: 
            self.remove_LRU()
        self.cache.append(msg)
        self.event_scheduler.schedule_event(msg.duration, self.expire_callback,False,args = [msg])
        self.get_logger().info('Message received')


        #self.get_logger().info(f'TIME:(sec:{msg.timestamp.sec},nanosec:{msg.timestamp.nanosec}), DATA: {msg.data}')
        self.print_cache()

    def remove_FIFO(self):
        temp_msg=self.cache[0]
        try:
            self.cache.pop(0)
            self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            self.get_logger().info(f'ERR_FIFO: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass
            
    def remove_RND(self):
        temp_msg=self.cache[random.randint(0, len(self.cache) - 1)]

        try:
            self.cache.remove(temp_msg)
            self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            self.get_logger().info(f'ERR_RND: {temp_msg.sensor_id}-{temp_msg.sqn}')       
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
            self.get_logger().info(f'Removing: {temp_msg.sensor_id}-{temp_msg.sqn}')
            
        except ValueError:
            self.get_logger().info(f'ERR_LRU: {temp_msg.sensor_id}-{temp_msg.sqn}')
            pass

    def expire_callback(self,msg):
        try:
            self.cache.remove(msg)
            self.get_logger().info(f'Expired: {msg.sensor_id}-{msg.sqn}')
        except ValueError:
            self.get_logger().info(f'ERR_EXP: {msg.sensor_id}-{msg.sqn}')
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

        #self.get_logger().info(f"Action requested. Performing movement to targets:\n\t{command_goal.targets}")

        self.fly_to_altitude(MIN_ALTITUDE_TO_PERFORM_PATROL)


        targets_patrolled = 0
        
        for target in command_goal.targets:
                
            self.rotate_to_target(target)
            self.move_to_target(target)

            #self.get_logger().info(f"Movement to target {targets_patrolled} completed!")
            targets_patrolled += 1
        
        
        goal.succeed()

        result =  Patrol.Result()
        result.result = "Movement completed"

        return result

    def fly_to_altitude(self, altitude):

        # Instantiate the move_up message
        move_up = Twist()
        move_up.linear = Vector3(x=0.0, y=0.0, z=1.0)
        move_up.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.cmd_vel_publisher.publish(move_up)

        # Wait for the drone to reach the desired altitude
        while(self.position.z < altitude):
            time.sleep(0.1)

        # Stop movement after the altitude has been reached
        stop_mov = Twist()
        stop_mov.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_mov.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_mov)

    def rotate_to_target(self, target, eps = 0.5):

        # We compute the angle between the current target position and the target
        # position here
        target_angle = math_utils.angle_between_points(self.position, target)
        angle_to_rotate = target_angle - self.yaw

        # Normalize the angle difference to be within the range [-pi, pi]
        angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi

        # And then assign the direction of the rotation correctly
        rotation_dir = 1 if angle_to_rotate < 0 else -1
        
        # Prepare the cmd_vel message
        move_msg = Twist()
        move_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z = 0.5 * rotation_dir)
        self.cmd_vel_publisher.publish(move_msg)

        # Publish the message until the correct rotation is reached (accounting for some eps error)
        # Note that here the eps also helps us stop the drone and not overshoot the target, as
        # the drone will keep moving for a while after it receives a stop message
        # Also note that rotating the drone too fast will make it loose altitude.
        # You can account for that by also giving some z linear speed to the rotation movement.
        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.yaw

            # self.get_logger().info(f"Rotation: {self.yaw}")
            # No sleep here. We don't want to miss the angle by sleeping too much. Even 0.1 seconds
            # could make us miss the given epsilon interval

        # When done, send a stop message to be sure that the drone doesn't
        # overshoot its target
        stop_msg = Twist()
        stop_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_msg)

    def move_to_target(self, target, eps = 0.5, angle_eps = 0.02):


        # Save the target position and compute the distance
        distance = math_utils.point_distance(self.position, target)

        # Keep publishing movement while the distance is greater than the given EPS
        while (distance > eps):

            # Compute the move vector with the given position and target
            mv = math_utils.move_vector(self.position, target)

            twist_msg = Twist()
            twist_msg.linear.x = mv[0]
            twist_msg.linear.z = mv[1]

            # Check if Balloon is still facing the target correctly, otherwise add angular
            # velocity to the Twist msg
            target_angle = math_utils.angle_between_points(self.position, target)

            if not (target_angle - angle_eps < self.yaw < target_angle + angle_eps):
                angle_diff = (self.yaw - target_angle)
                twist_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_diff))


            # Publish msg
            self.cmd_vel_publisher.publish(twist_msg)

            # Update position and distance after finishing
            distance = math_utils.point_distance(self.position, target)


        # After reaching the target, publish a stop msg
        self.cmd_vel_publisher.publish(self.stop_msg)


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
