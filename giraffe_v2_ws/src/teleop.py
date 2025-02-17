import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import serial
import time
import math

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

        # NEW: Odometry publisher on /raw_odom topic
        self.odom_publisher = self.create_publisher(Odometry, '/raw_odom', 10)

        # Serial Communication Setup
        self.serial_port = "/dev/ttyUSB1"
        self.baudrate = 115200
        self.timeout = 1

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            self.get_logger().info(f"🚀 Connected to Serial Port: {self.serial_port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to {self.serial_port}: {e}")
            self.ser = None

        # Differential Drive Parameters
        self.wheel_diameter = 0.0955   # 9.55 cm wheel diameter
        self.base_width = 0.286         # 28.6 cm between wheels
        self.wheel_radius = self.wheel_diameter / 2

        # Acceleration and Deceleration limits (m/s^2 and rad/s^2)
        self.max_linear_accel = 1.0    
        self.max_linear_decel = 2.0    
        self.max_angular_accel = 1.5   
        self.max_angular_decel = 3.0   

        # Last known commanded velocities (for cmd_vel callback)
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_time = time.time()
        self.initialized = False
        self.last_cmd_time = time.time()

        # Variables for odometry integration
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = time.time()

        # Timer to check for cmd_vel timeout and read odometry from serial
        self.create_timer(1/30, self.check_cmd_vel_timeout)
        self.create_timer(1/30, self.read_odometry)

    def cmd_vel_callback(self, msg):
        """Callback to process cmd_vel and send wheel speeds over serial."""
        current_time = time.time()
        self.last_cmd_time = current_time

        if not self.initialized:
            self.last_time = current_time
            self.last_linear_x = 0.0
            self.last_angular_z = 0.0
            self.initialized = True

        dt = current_time - self.last_time
        self.last_time = current_time

        target_linear_x = msg.linear.x
        target_angular_z = msg.angular.z

        # Apply acceleration/deceleration limits for linear velocity
        delta_linear = target_linear_x - self.last_linear_x
        if delta_linear > 0:
            delta_linear = min(delta_linear, self.max_linear_accel * dt)
        else:
            delta_linear = max(delta_linear, -self.max_linear_decel * dt)
        linear_x = self.last_linear_x + delta_linear

        # Apply limits for angular velocity
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
        """Decelerate to zero if no cmd_vel messages are received."""
        current_time = time.time()
        dt = current_time - self.last_time
        timeout = 2/30

        if current_time - self.last_cmd_time > timeout:
            # Decelerate linear velocity
            if self.last_linear_x > 0:
                linear_x = max(self.last_linear_x - self.max_linear_decel * dt, 0.0)
            elif self.last_linear_x < 0:
                linear_x = min(self.last_linear_x + self.max_linear_decel * dt, 0.0)
            else:
                linear_x = 0.0

            # Decelerate angular velocity
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
        """
        Read odometry data from the motor controller over serial.
        The controller sends wheel velocities (left and right in m/s) as a string.
        This function integrates these velocities to update the robot's pose and publishes an Odometry message.
        """
        if self.ser:
            data = self.ser.readline().decode().strip()
            if data:
                # self.get_logger().info(f"Raw odometry data: {data}")
                try:
                    # Replace commas with spaces and split into two values
                    data = data.replace(',', ' ')
                    v_left, v_right = map(float, data.split())
                    
                    # Compute the robot's linear and angular velocities
                    linear_vel = (v_left + v_right) / 2.0
                    angular_vel = (v_right - v_left) / self.base_width

                    # Time delta for integration
                    current_odom_time = time.time()
                    dt = current_odom_time - self.last_odom_time
                    self.last_odom_time = current_odom_time

                    # Integrate to update pose
                    self.x += linear_vel * math.cos(self.yaw) * dt
                    self.y += linear_vel * math.sin(self.yaw) * dt
                    self.yaw += angular_vel * dt

                    # Create quaternion from yaw (since roll and pitch are zero)
                    qz = math.sin(self.yaw / 2.0)
                    qw = math.cos(self.yaw / 2.0)
                    odom_quat = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

                    # Build the Odometry message
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link_real'

                    # Set the position and orientation
                    odom_msg.pose.pose.position.x = self.x
                    odom_msg.pose.pose.position.y = self.y
                    odom_msg.pose.pose.position.z = 0.0
                    odom_msg.pose.pose.orientation = odom_quat

                    # Set the velocities
                    odom_msg.twist.twist.linear.x = linear_vel
                    odom_msg.twist.twist.angular.z = angular_vel

                    # Publish the odometry message
                    self.odom_publisher.publish(odom_msg)
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
            self.ser.flush()
            # self.get_logger().info(f"Sent to serial: {cmd.strip()}")
        else:
            self.get_logger().warn(f"❌ Serial port not available, vel: {cmd}")

    def shutdown(self):
        """Gracefully shutdown and close the serial port."""
        if self.ser:
            self.ser.close()
            self.get_logger().info("🔌 Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard Interrupt: Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
