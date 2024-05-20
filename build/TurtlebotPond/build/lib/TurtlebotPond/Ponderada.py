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

app = typer.Typer(help="Robot Controller Interface")

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_client = self.create_client(Empty, 'emergency_stop')
        self.connected = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def print_status(self):
        status = "Connected" if self.connected else "Disconnected"
        print(f"Status: {status}, Linear Speed: {self.linear_speed:.2f} m/s, Angular Speed: {self.angular_speed:.2f} rad/s")

    def connect(self):
        if not self.connected:
            self.connected = True
            print("Robot connected and ready to publish.")
        self.print_status()

    def disconnect(self):
        if self.connected:
            self.connected = False
            print("Robot disconnected, please connect again to use.")
        self.print_status()

    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
        self.print_status()

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Stopping robot.")
        self.print_status()

    def update_speed(self, linear, angular):
        self.linear_speed += linear
        self.angular_speed += angular
        self.move_robot()

    def send_emergency_stop(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.emergency_client.call_async(req)
            future.add_done_callback(self.emergency_stop_callback)
        else:
            print('Emergency service not available, waiting...')

    def emergency_stop_callback(self, future):
        try:
            future.result()
            print('Emergency stop signal sent successfully.')
        except Exception as e:
            print(f'Failed to call emergency stop service: {e}')

    def kill_switch(self):
        print("Emergency process stop forced.")
        self.stop_robot()
        self.send_emergency_stop()
        rclpy.shutdown()

@app.command()
def teleop():
    """ Enter teleoperation mode to control the robot manually. """
    rclpy.init()
    robot_controller = RobotController()

    choices = [
        {'name': 'Connect', 'function': robot_controller.connect},
        {'name': 'Disconnect', 'function': robot_controller.disconnect},
        {'name': 'Move', 'function': lambda: manual_control(robot_controller)},
        {'name': 'Stop', 'function': robot_controller.stop_robot},
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

def manual_control(robot_controller):
    print("Manual control activated. Use 'w', 's', 'a', 'd' to move, 'q' to return.")
    while True:
        key = get_key()
        if key == 'q':
            break
        actions = {'w': (0.1, 0), 's': (-0.1, 0), 'a': (0, 0.1), 'd': (0, -0.1)}
        if key in actions:
            robot_controller.update_speed(*actions[key])
        time.sleep(0.1)

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

def exit_program(robot_controller):
    robot_controller.disconnect()
    rclpy.shutdown()
    sys.exit(0)

def main():
    teleop()

if __name__ == '__main__':
    main()
