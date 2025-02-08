import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
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

        # ROS2 Publisher for odometry
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
        self.wheel_diameter = 0.10  # 10cm -> 0.10m
        self.base_width = 0.24  # 24cm -> 0.24m
        self.wheel_radius = self.wheel_diameter / 2  # Radius of the wheel

        # Acceleration limits (m/s^2)
        self.max_linear_accel = 0.5  # Max linear acceleration
        self.max_angular_accel = 1.0  # Max angular acceleration

        # Last known velocities
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_time = time.time()
        self.initialized = False
        self.last_cmd_time = time.time()

        # Timer to check for cmd_vel timeout and read odometry
        self.create_timer(0.1, self.check_cmd_vel_timeout)
        self.create_timer(0.1, self.read_odometry)

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

        target_linear_x = msg.linear.x  # Target forward velocity (m/s)
        target_angular_z = msg.angular.z  # Target rotational velocity (rad/s)

        # Apply acceleration limits
        linear_x = self.last_linear_x + max(min(target_linear_x - self.last_linear_x, self.max_linear_accel * dt), -self.max_linear_accel * dt)
        angular_z = self.last_angular_z + max(min(target_angular_z - self.last_angular_z, self.max_angular_accel * dt), -self.max_angular_accel * dt)

        self.last_linear_x = linear_x
        self.last_angular_z = angular_z

        self.send_wheel_velocities(linear_x, angular_z)

    def check_cmd_vel_timeout(self):
        """Check if no cmd_vel message is received and decelerate to zero."""
        current_time = time.time()
        dt = current_time - self.last_time
        timeout = 0.1  # Timeout duration in seconds

        if current_time - self.last_cmd_time > timeout:
            linear_x = max(self.last_linear_x - self.max_linear_accel * dt, 0.0) if self.last_linear_x > 0 else min(self.last_linear_x + self.max_linear_accel * dt, 0.0)
            angular_z = max(self.last_angular_z - self.max_angular_accel * dt, 0.0) if self.last_angular_z > 0 else min(self.last_angular_z + self.max_angular_accel * dt, 0.0)

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
                    # odom_msg = Odometry()
                    # odom_msg.twist.twist.linear.x = (left_vel + right_vel) / 2
                    # odom_msg.twist.twist.angular.z = (right_vel - left_vel) / self.base_width
                    # self.odom_publisher.publish(odom_msg)
                except ValueError:
                    self.get_logger().warn(f"⚠️ Invalid odometry data received: {data}")

    def send_wheel_velocities(self, linear_x, angular_z):
        """Calculate and send wheel velocities over serial."""
        left_wheel_vel = (linear_x - (angular_z * self.base_width / 2))
        right_wheel_vel = (linear_x + (angular_z * self.base_width / 2))

        cmd = f"{left_wheel_vel:.2f} {right_wheel_vel:.2f}\n"

        if self.ser:
            self.ser.write(cmd.encode())
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
