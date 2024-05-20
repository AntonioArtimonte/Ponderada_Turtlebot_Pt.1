import typer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import inquirer
import sys
import os
import time

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

app = typer.Typer(help="Robot Controller Interface (CLI)")

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_client = self.create_client(Empty, 'emergency_stop') # New ROS2 topic made to stop execution of the bringup in the Robot
        self.connected = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    # Method to print the robot status
    def print_status(self):
        status = "Connected" if self.connected else "Disconnected"
        print(f"Status: {status}, Linear Speed: {self.linear_speed:.2f} m/s, Angular Speed: {self.angular_speed:.2f} rad/s")

    # Function to "connect" the robot. Doesn't actually connect to anything, just sets the connected var to True to prevent any mistakes in the operation 
    def connect(self):
        if not self.connected:
            self.connected = True
            print("Robot connected and ready to publish.")
        self.print_status()

    # Function to "disconnect" the robot. Doesn't actually disconnect from anything, just sets the connected var to False to prevent any mistakes in the operation
    def disconnect(self):
        if self.connected:
            self.connected = False
            print("Robot disconnected, please connect again to use.")
        self.print_status()

    # Function to move the robot, if it's connected. Will get the linear and angular speed from the class and publish it to the cmd_vel topic
    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
        self.print_status()

    # Stop robot function, will just ask function move_robot with linear and angular speed to 0
    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Stopping robot.")
        self.print_status()

    # Function to update the robot speed, based on it's current speed and the keyboard input to add more speed in any of the axis
    def update_speed(self, linear, angular):
        self.linear_speed += linear
        self.angular_speed += angular
        self.move_robot()

    # Function to send the emergency stop signal to the robot, will call the emergency_stop service that will kill the robot_bringup process inside the robot. After pressing it, the robot will need a restart to work again.
    def send_emergency_stop(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.emergency_client.call_async(req)
            future.add_done_callback(self.emergency_stop_callback)
        else:
            print('Emergency service not available, SHUTDOWN THE ROBOTTTT')

    # Function to handle the callback of the emergency stop service, will print a message if the service was successfull or not.
    def emergency_stop_callback(self, future):
        try:
            future.result()
            print('Emergency stop signal sent successfully, robot process terminated.')
        except Exception as e:
            print(f'Failed to call emergency stop service: {e}, REMOVE THE BATTERY FROM THE ROBOT!!!')

    # Function to shutdown the ros2 node, and call the "send_emergency_stop" function to completely stop the robot process
    def kill_switch(self):
        print("Emergency process stop forced.")
        self.stop_robot()
        self.send_emergency_stop()
        rclpy.shutdown()

# Main function to start the teleoperation of the robot, will create the RobotController class and start the teleoperation interface
@app.command()
def teleop():
    """ Enter teleoperation mode to control the robot manually !!!MUST BE CONNECTED FIRST!!!. """
    rclpy.init()
    robot_controller = RobotController()

    choices = [
        {'name': 'Connect', 'function': robot_controller.connect},
        {'name': 'Disconnect', 'function': robot_controller.disconnect},
        {'name': 'Move', 'function': lambda: manual_control(robot_controller)},
        {'name': 'Emergency Stop', 'function': robot_controller.kill_switch},
        {'name': 'Exit', 'function': lambda: exit_program(robot_controller)}
    ]

    while True:
        action = inquirer.prompt([
            inquirer.List('action',
                          message="Choose an action:",
                          choices=[choice['name'] for choice in choices])
        ])

        for choice in choices:
            if choice['name'] == action['action']:
                choice['function']()
                break

# Function to control the robot based on the clicked key.
def manual_control(robot_controller):
    print("Manual control activated. Use 'w', 's', 'a', 'd' to move, 'q' to return and 'b' to emergency stop.")
    while True:
        key = get_key()
        if key == 'q':
            break
        actions = {'w': (0.1, 0), 's': (-0.1, 0), 'a': (0, 0.1), 'd': (0, -0.1)}
        if key in actions:
            robot_controller.update_speed(*actions[key])
        time.sleep(0.1)
        if key == 'b':
            robot_controller.kill_switch

# Function to get the clicked key
def get_key():
    if os.name == 'nt':
        return msvcrt.getch().decode() if msvcrt.kbhit() else ''
    else:
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        try:
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

# Function to exit the CLI. Will disconnect the robot and shutdown the ros2 node
def exit_program(robot_controller):
    robot_controller.disconnect()
    rclpy.shutdown()
    sys.exit(0)

# Main function that only calls the teleop function.
def main():
    teleop()

if __name__ == '__main__':
    main()