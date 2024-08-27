import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.executors import MultiThreadedExecutor
import time
import math
from project_interfaces.msg import Data
from rosgraph_msgs.msg import Clock
import math_utils
from sim_utils import EventScheduler
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
from project_interfaces.action import Patrol
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
WORLD_NAME = "iot_project_world"

class SensorController(Node):

    def __init__(self):
        super().__init__('sensor_controller')
        self.position = Point(x = 0.0, y = 0.0, z = 0.0)
        self.yaw = 0

        self.stop_msg = Twist()
        self.patrol_action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            self.execute_patrol_action
        )
        
        self.odometry_subscrber = self.create_subscription(
            Odometry,
            'odometry',
            self.store_position,
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.tx_topic = self.create_publisher(
            Data,
            'tx_data',
            10
        )
        
        self.id = self.declare_parameter('id', -1)

        self.generated_data = 0


        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )
        self.rate = 0.2 #average num of msg per second
        self.event_scheduler.schedule_event(np.random.exponential(1 / self.rate), self.simple_publish, False)

        #self.create_timer(1, self.simple_publish)


    def simple_publish(self):
        id = self.id.get_parameter_value().integer_value

        msg = Data()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.duration = 10#one min
        msg.sensor_id = id
        msg.sqn = self.generate_data()
        msg.data = f"Sensor data: {id}_{msg.sqn}!"

        self.tx_topic.publish(msg)
        self.event_scheduler.schedule_event(np.random.exponential(1 / self.rate), self.simple_publish, False) 

    def execute_patrol_action(self, goal : ServerGoalHandle):


        command_goal : Patrol.Goal = goal.request

        self.get_logger().info(f"Action requested. Performing movement to targets:\n\t{command_goal.targets}")



        targets_patrolled = 0
        self.get_logger().info(f"target {command_goal.targets}")
        for target in command_goal.targets:
                
            self.rotate_to_target(target)
            self.move_to_target(target)

            self.get_logger().info(f"Movement to target {targets_patrolled} completed!")
            targets_patrolled += 1
        self.get_logger().info(f"completed")
        
        goal.succeed()

        result =  Patrol.Result()
        result.result = "Movement completed"

        return result


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

    def move_to_target(self, target, eps = 1, angle_eps = 0.02):


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


    def store_position(self, odometry_msg : Odometry):

        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )
    def generate_data(self):

        self.generated_data += 1
        return self.generated_data


def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor()

    sensor_controller = SensorController()
    executor.add_node(sensor_controller)

    executor.spin()

    sensor_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
