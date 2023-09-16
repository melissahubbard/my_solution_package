import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import math
import random

class TurtleChaser(Node):

    def __init__(self):
        super().__init__('m_h_tc')
        self.target_turtle_name = "turtle2"
        self.target_turtle_pose = None

        self.declare_parameter('catch_distance', 0.5)

        self.target_turtle_subscription = self.create_subscription(
            Pose,
            f'/{self.target_turtle_name}/pose',
            self.target_turtle_callback,
            10)

        self.turtle1_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')

        self.spawn_new_target()

    def spawn_new_target(self):
        self.get_logger().info('Attempting to spawn a new turtle.')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        req = Spawn.Request()
        req.x = random.uniform(0, 10)
        req.y = random.uniform(0, 10)
        req.theta = random.uniform(0, 6.28319)  # 0 to 2*pi (360 degrees)
        req.name = self.target_turtle_name

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_done)

    def spawn_done(self, future):
        response = future.result()
        if not response:
            self.get_logger().error("Failed to spawn a new turtle.")
        else:
            self.get_logger().info(f"Spawned a new turtle: {self.target_turtle_name}")

    def kill_target(self):
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        req = Kill.Request()
        req.name = self.target_turtle_name

        future = self.kill_client.call_async(req)
        future.add_done_callback(lambda fut: self.kill_done(fut))

    def kill_done(self, future):
        future.result()
        self.get_logger().info(f"Killed {self.target_turtle_name}")
        self.spawn_new_target()

    def target_turtle_callback(self, msg):
        self.target_turtle_pose = msg

    def turtle1_callback(self, msg):
        if self.target_turtle_pose is None:
            return

        distance = math.sqrt(
            (self.target_turtle_pose.x - msg.x)**2 + (self.target_turtle_pose.y - msg.y)**2
        )

        if distance < self.get_parameter('catch_distance').value:
            self.kill_target()
            return

        # Calculate the angle between the chasing turtle's orientation and the target direction
        target_angle = math.atan2(self.target_turtle_pose.y - msg.y, self.target_turtle_pose.x - msg.x)
        current_angle = msg.theta

        # Calculate the difference in angles and apply a smoothing factor (e.g., 0.2)
        angle_difference = target_angle - current_angle
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))  # Normalize angle
        angular_velocity = 4 * angle_difference

        # Adjust the linear velocity based on distance
        linear_velocity = distance

        # Check if the turtle is too close to the wall and correct its position
        if msg.x < 0.5 or msg.x > 10.5 or msg.y < 0.5 or msg.y > 10.5:
            # Calculate the angle to the center of the world
            angle_to_center = math.atan2(5 - msg.y, 5 - msg.x)
            angular_velocity += angle_to_center  # Rotate towards the center

        velocity_msg = Twist()
        velocity_msg.linear.x = linear_velocity
        velocity_msg.angular.z = angular_velocity

        self.publisher_.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)

    node = TurtleChaser()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
