import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class DiffDriveSerial(Node):
    def __init__(self):
        super().__init__('diff_drive_serial')

        # ROS2 Subscriber to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10  # Queue size
        )

        # ROS2 Publisher for odometry (commented out)
        # self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Serial Communication Setup
        self.serial_port = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.timeout = 1

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            self.get_logger().info(f"🚀 Connected to Serial Port: {self.serial_port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to {self.serial_port}: {e}")
            self.ser = None

        # Differential Drive Parameters
        self.wheel_diameter = 0.0955   # e.g., 9.55cm wheel diameter
        self.base_width = 0.24         # 24cm between wheels
        self.wheel_radius = self.wheel_diameter / 2

        # Acceleration and Deceleration limits (m/s^2 and rad/s^2)
        self.max_linear_accel = 0.5    # maximum linear acceleration
        self.max_linear_decel = 0.7    # maximum linear deceleration (can be higher)
        self.max_angular_accel = 1.0   # maximum angular acceleration
        self.max_angular_decel = 1.2   # maximum angular deceleration

        # Last known velocities
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_time = time.time()
        self.initialized = False
        self.last_cmd_time = time.time()

        # Timer to check for cmd_vel timeout and read odometry
        self.create_timer((1/30), self.check_cmd_vel_timeout)
        self.create_timer((1/30), self.read_odometry)

    def cmd_vel_callback(self, msg):
        """Callback function to process cmd_vel and send wheel speeds over serial."""
        current_time = time.time()
        self.last_cmd_time = current_time

        if not self.initialized:
            self.last_time = current_time
            self.last_linear_x = 0.0
            self.last_angular_z = 0.0
            self.initialized = True

        dt = current_time - self.last_time
        self.last_time = current_time

        target_linear_x = msg.linear.x   # Target forward velocity (m/s)
        target_angular_z = msg.angular.z   # Target rotational velocity (rad/s)

        # Apply separate acceleration or deceleration limits for linear velocity
        delta_linear = target_linear_x - self.last_linear_x
        if delta_linear > 0:
            # Accelerating
            delta_linear = min(delta_linear, self.max_linear_accel * dt)
        else:
            # Decelerating
            delta_linear = max(delta_linear, -self.max_linear_decel * dt)
        linear_x = self.last_linear_x + delta_linear

        # Apply separate acceleration or deceleration limits for angular velocity
        delta_angular = target_angular_z - self.last_angular_z
        if delta_angular > 0:
            delta_angular = min(delta_angular, self.max_angular_accel * dt)
        else:
            delta_angular = max(delta_angular, -self.max_angular_decel * dt)
        angular_z = self.last_angular_z + delta_angular

        self.last_linear_x = linear_x
        self.last_angular_z = angular_z

        self.send_wheel_velocities(linear_x, angular_z)

    def check_cmd_vel_timeout(self):
        """If no cmd_vel message is received for a timeout period, decelerate to zero using deceleration limits."""
        current_time = time.time()
        dt = current_time - self.last_time
        # Timeout duration (for example, 2/30 seconds)
        timeout = 2/30

        if current_time - self.last_cmd_time > timeout:
            # Decelerate linear velocity to 0
            if self.last_linear_x > 0:
                linear_x = max(self.last_linear_x - self.max_linear_decel * dt, 0.0)
            elif self.last_linear_x < 0:
                linear_x = min(self.last_linear_x + self.max_linear_decel * dt, 0.0)
            else:
                linear_x = 0.0

            # Decelerate angular velocity to 0
            if self.last_angular_z > 0:
                angular_z = max(self.last_angular_z - self.max_angular_decel * dt, 0.0)
            elif self.last_angular_z < 0:
                angular_z = min(self.last_angular_z + self.max_angular_decel * dt, 0.0)
            else:
                angular_z = 0.0

            self.last_linear_x = linear_x
            self.last_angular_z = angular_z

            self.send_wheel_velocities(linear_x, angular_z)

    def read_odometry(self):
        """Reads odometry data from the motor controller and publishes it."""
        if self.ser:
            data = self.ser.readline().decode().strip()
            if data:
                self.get_logger().info(f"#####################Odometry: {data}")
                try:
                    data = data.replace(',', ' ')
                    left_vel, right_vel = map(float, data.split())
                    # Here you could publish an Odometry message if desired.
                except ValueError:
                    self.get_logger().warn(f"⚠️ Invalid odometry data received: {data}")

    def send_wheel_velocities(self, linear_x, angular_z):
        """Calculate and send wheel velocities over serial."""
        left_wheel_vel = linear_x - (angular_z * self.base_width / 2)
        right_wheel_vel = linear_x + (angular_z * self.base_width / 2)

        # Format the command with start and end markers.
        cmd = f"<{left_wheel_vel:.1f} {right_wheel_vel:.1f}>"

        if self.ser:
            self.ser.write(cmd.encode())
            self.ser.flush()  # Flush the serial buffer
            self.get_logger().info(f"Sent to serial: {cmd.strip()}")
        else:
            self.get_logger().warn(f"❌ Serial port not available, vel: {cmd}")

    def shutdown(self):
        """Graceful shutdown to close the serial port."""
        if self.ser:
            self.ser.close()
            self.get_logger().info("🔌 Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSerial()

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard Interrupt: Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
