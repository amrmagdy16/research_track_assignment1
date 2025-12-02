import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UI_Node(Node):
    def __init__(self):
        super().__init__('ui_node')
        # Publishers for both turtles
        self.pub_turtle1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_turtle2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.get_logger().info("UI Node Started. Ready for input.")

    def move_turtle(self, publisher, lin_vel, ang_vel):
        # Create the move command
        msg = Twist()
        msg.linear.x = float(lin_vel)
        msg.angular.z = float(ang_vel)

        # Publish start command
        publisher.publish(msg)
        self.get_logger().info(f"Moving...")

        # Wait for 1 second (Assignment Requirement)
        time.sleep(1.0)

        # Create the stop command
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        publisher.publish(msg)
        self.get_logger().info("Stopped.")

    def run_interface(self):
        while rclpy.ok():
            print("\n--- Turtle Controller ---")
            try:
                # 1. Select Robot
                t_id = input("Select Turtle (1 or 2): ")
                if t_id not in ['1', '2']:
                    print("Invalid ID. Please enter 1 or 2.")
                    continue

                # 2. Select Velocity
                lin = input("Linear Velocity (x): ")
                ang = input("Angular Velocity (z): ")

                # 3. Choose Publisher
                active_pub = self.pub_turtle1 if t_id == '1' else self.pub_turtle2

                # 4. Execute
                self.move_turtle(active_pub, lin, ang)

            except ValueError:
                print("Invalid input. Please enter numbers.")
            except Exception as e:
                print(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UI_Node()
    try:
        node.run_interface()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
