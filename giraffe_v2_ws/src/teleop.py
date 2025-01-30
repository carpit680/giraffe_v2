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

    def cmd_vel_callback(self, msg):
        """Callback function to process cmd_vel and send wheel speeds over serial."""
        linear_x = msg.linear.x  # Forward velocity (m/s)
        angular_z = msg.angular.z  # Rotational velocity (rad/s)

        # Calculate wheel velocities using differential drive kinematics
        left_wheel_vel = (linear_x - (angular_z * self.base_width / 2)) / self.wheel_radius
        right_wheel_vel = (linear_x + (angular_z * self.base_width / 2)) / self.wheel_radius

        # Convert to RPM (if required by motor controller)
        left_wheel_rpm = (left_wheel_vel * 60) / (2 * 3.1416)  # rad/s to RPM
        right_wheel_rpm = (right_wheel_vel * 60) / (2 * 3.1416)  # rad/s to RPM

        # Format the command as "<left_rpm> <right_rpm>\n"
        cmd = f"{left_wheel_rpm:.2f} {right_wheel_rpm:.2f}\n"

        if self.ser:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"Sent to serial: {cmd.strip()}")
        else:
            self.get_logger().warn("❌ Serial port not available.")

    def read_odometry(self):
        """Reads odometry data from the motor controller."""
        while rclpy.ok():
            if self.ser:
                data = self.ser.readline().decode().strip()
                if data:
                    self.get_logger().info(f"Odometry: {data}")
            time.sleep(0.1)  # Read frequency

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
