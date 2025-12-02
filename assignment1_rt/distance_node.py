import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class DistanceMonitor(Node):
    def __init__(self):
        super().__init__('distance_node')

        # Storage for turtle positions
        self.pose1 = None
        self.pose2 = None

        # Subscribers to get positions
        self.create_subscription(Pose, '/turtle1/pose', self.pose1_cb, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_cb, 10)

        # Publishers for distance and emergency stop
        self.pub_dist = self.create_publisher(Float32, '/turtle_distance', 10)
        self.pub_stop1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_stop2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Timer to run the check loop 10 times per second
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose1_cb(self, msg): self.pose1 = msg
    def pose2_cb(self, msg): self.pose2 = msg

    def control_loop(self):
        # Wait until we have data from both turtles
        if self.pose1 is None or self.pose2 is None:
            return

        # 1. Calculate Distance (Euclidean)
        dist = math.sqrt((self.pose1.x - self.pose2.x)**2 + (self.pose1.y - self.pose2.y)**2)

        # 2. Publish Distance
        dist_msg = Float32()
        dist_msg.data = dist
        self.pub_dist.publish(dist_msg)

        # 3. Define Safety Thresholds
        SAFE_DIST = 1.5 
        BOUND_MAX = 10.0
        BOUND_MIN = 1.0

        # 4. Check Conditions & Stop if necessary
        # Condition A: Too Close to each other
        if dist < SAFE_DIST:
            self.get_logger().warn(f"Too Close! Distance: {dist:.2f}")
            self.stop_turtles()

        # Condition B: Too close to wall (Turtle 1)
        if (self.pose1.x > BOUND_MAX or self.pose1.x < BOUND_MIN or
            self.pose1.y > BOUND_MAX or self.pose1.y < BOUND_MIN):
            self.get_logger().warn(f"Turtle 1 hitting wall!")
            self.stop_turtles()

        # Condition B: Too close to wall (Turtle 2)
        if (self.pose2.x > BOUND_MAX or self.pose2.x < BOUND_MIN or
            self.pose2.y > BOUND_MAX or self.pose2.y < BOUND_MIN):
            self.get_logger().warn(f"Turtle 2 hitting wall!")
            self.stop_turtles()

    def stop_turtles(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.pub_stop1.publish(stop_msg)
        self.pub_stop2.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
