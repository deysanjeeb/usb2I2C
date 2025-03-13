# Insert license


# Partially based on https://github.com/flynneva/bno055/blob/main/bno055/bno055.py

import sys
import threading
import time
import glob 

# Import the BNO085Bridge from the second file
from bno085.bridge.BNO085Bridge import BNO085Bridge

from bno085.sensor.SensorService import SensorService
from bno085.params.NodeParameters import NodeParameters

import rclpy
from rclpy.node import Node


class Bno085Node(Node):
    """
    ROS2 Node for interfacing Bosch BNO085 IMU sensor.

    :param Node: ROS2 Node Class to initialize from
    :type Node: ROS2 Node
    :raises NotImplementedError: Indicates feature/function is not implemented yet.
    """

    sensor = None
    param = None
    bridge = None

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('bno085')

    def setup(self):
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:
        if self.param.connection_type.value == "i2c":
            # Instead of using board.I2C(), we'll use the BNO085Bridge
            # Declare the port parameter with a default value
            import glob
            available_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            
            if not available_ports:
                self.get_logger().error('No Arduino ports found!')
                raise RuntimeError('No Arduino ports found!')
                
            # Use the first available port or allow parameter override
            self.declare_parameter('arduino_port', available_ports[1])
            port = self.get_parameter('arduino_port').value
            # self.declare_parameter('arduino_port', '/dev/ttyACM1')
            # port = self.get_parameter('arduino_port').value
            
            self.get_logger().info(f'Connecting to BNO085 via Arduino bridge on port {port}')
            
            # Create and connect to the Arduino bridge
            self.bridge = BNO085Bridge(port=port)
            if not self.bridge.connect():
                self.get_logger().error('Failed to connect to Arduino bridge')
                raise RuntimeError('Failed to connect to Arduino bridge')
                
            # Initialize the sensor
            if not self.bridge.initialize_bno085():
                self.get_logger().error('Failed to initialize BNO085 sensor')
                raise RuntimeError('Failed to initialize BNO085 sensor')
                
            # Enable rotation vector reports
            self.bridge.enable_rotation_vector()
            
            # Create a wrapper object that presents the bridge as if it were a BNO08X_I2C object
            connector = BridgeToAdafruitAdapter(self.bridge, self)
        else:
            raise NotImplementedError('Unsupported connection type: '
                                     + str(self.param.connection_type.value))

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()


class BridgeToAdafruitAdapter:
    """
    Adapter class that makes the BNO085Bridge appear like an Adafruit BNO08X_I2C object
    to the SensorService class, allowing easy integration.
    """
    
    def __init__(self, bridge, node):
        """Initialize with a BNO085Bridge object"""
        self.bridge = bridge
        self.node = node
        self._quaternion = (0.0, 0.0, 0.0, 1.0)  # Default quaternion (x, y, z, w)
        self._acceleration = (0.0, 0.0, 0.0)  # Default acceleration (x, y, z)
        self._gyro = (0.0, 0.0, 0.0)  # Default gyro (x, y, z)
        self._magnetic = (0.0, 0.0, 0.0)  # Default magnetic (x, y, z)
        self._linear_acceleration = (0.0, 0.0, 0.0)  # Default linear acceleration (x, y, z)
        self._calibration_status = 3   # Default status (fully calibrated)
        
    def initialize(self):
        """Mock initialize method"""
        return True
    
    def update(self):
        """Update sensor data by reading from the bridge"""
        quat = self.bridge.get_quaternion()
        if quat:
            try:
        # Explicitly convert to float to ensure proper type
                self._quaternion = (
                    float(quat['qx']), 
                    float(quat['qy']), 
                    float(quat['qz']), 
                    float(quat['qw'])
                )
                self.node.get_logger().debug(f"Updated quaternion: {self._quaternion}")
                return True
            except (ValueError, TypeError) as e:
                self.node.get_logger().error(f"Error converting quaternion values: {e}")
                self.node.get_logger().error(f"Raw quaternion data: {quat}")
                return False
        return False
        
    @property
    def quaternion(self):
        """Return the quaternion data"""
        # Ensure we have the latest data
        self.update()
        return self._quaternion

    @property
    def calibration_status(self):
        """Return the calibration status"""
        return self._calibration_status
    
    
    @property
    def acceleration(self):
        """Return acceleration data - this is a placeholder as we don't get this from the bridge"""
        return self._acceleration

    @property
    def linear_acceleration(self):
        """Return linear acceleration data - this is a placeholder as we don't get this from the bridge"""
        return self._linear_acceleration
    
    @property
    def gyro(self):
        """Return gyro data - this is a placeholder as we don't get this from the bridge"""
        return self._gyro
    
    @property
    def magnetic(self):
        """Return magnetic data - this is a placeholder as we don't get this from the bridge"""
        return self._magnetic
    
    def enable_feature(self, feature):
        """Mock enable_feature method - the bridge already enables what we need"""
        self.node.get_logger().info(f"Enabling feature {feature} (mocked)")
        pass
    
    def begin_calibration(self):
        """Mock begin_calibration method"""
        self.node.get_logger().info("Beginning calibration (mocked)")
        # Send calibration command to bridge if the bridge supports it
        # Otherwise just log that we're mocking this functionality
        return True
    
    def save_calibration_data(self):
        """Mock save_calibration method"""
        self.node.get_logger().info("Saving calibration (mocked)")
        return True
    
    def get_calibration(self):
        """Mock get_calibration method"""
        # Return a mock calibration status (3 = fully calibrated)
        return self._calibration_status
    
    def get_calibration_status(self):
        """Mock get_calibration_status method"""
        # Return a mock calibration status (3 = fully calibrated)
        return self._calibration_status
    
    # Add any other methods that might be required by SensorService


def main(args=None):
    try:
        """Main entry method for this ROS2 node."""
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = Bno085Node()
        node.setup()

        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()

        def read_data():
            """Periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except ZeroDivisionError:
                # division by zero in get_sensor_data, return
                return
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving sensor data failed with %s:"%s"'
                                       % (type(e).__name__, e))
            finally:
                lock.release()

        def log_calibration_status():
            """Periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving calibration status failed with %s:"%s"'
                                       % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()

        # start regular sensor transmissions:
        # please be aware that frequencies around 30Hz and above might cause performance impacts:
        # https://github.com/ros2/rclpy/issues/520
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
        except UnboundLocalError:
            node.get_logger().info('No timers to shutdown')
        # Close the bridge connection
        if hasattr(node, 'bridge') and node.bridge is not None:
            node.bridge.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()