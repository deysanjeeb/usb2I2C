#!/usr/bin/env python3
import serial
import time
import sys
import re
import signal
from datetime import datetime

class BNO085Bridge:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        """Initialize connection to Arduino I2C bridge for BNO085"""
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.running = False
        
    def connect(self):
        """Connect to the Arduino bridge"""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=2)
            time.sleep(2)  # Give Arduino time to reset
            
            # Clear any startup messages
            self.ser.reset_input_buffer()
            print(f"Connected to Arduino bridge on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to Arduino: {e}")
            return False
            
    def send_command(self, command, wait_time=0.1):
        """Send a command to the Arduino and return any output"""
        if not self.ser:
            print("Not connected to Arduino")
            return []
            
        # Clear input buffer
        self.ser.reset_input_buffer()
        
        # Send command
        self.ser.write(command.encode())
        
        # Wait for processing
        time.sleep(wait_time)
        
        # Read all lines
        lines = []
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(line)
                lines.append(line)
            
        return lines
            
    def scan_i2c(self):
        """Scan the I2C bus for devices"""
        return self.send_command('S')
        
    def initialize_bno085(self):
        """Initialize the BNO085 sensor"""
        print("Initializing BNO085 sensor...")
        response = self.send_command('I', wait_time=0.5)
        return any("initialized successfully" in line for line in response)
        
    def enable_rotation_vector(self):
        """Enable rotation vector output from BNO085"""
        print("Enabling rotation vector...")
        return self.send_command('E', wait_time=0.5)
        
    def get_quaternion(self):
        """Get quaternion data from BNO085"""
        response = self.send_command('Q', wait_time=0.1)
        
        # Parse quaternion data
        for line in response:
            if line.startswith("Quaternion:"):
                # Extract values using regex
                match = re.search(r'qw=([\d.-]+) qx=([\d.-]+) qy=([\d.-]+) qz=([\d.-]+)', line)
                if match:
                    qw = float(match.group(1))
                    qx = float(match.group(2))
                    qy = float(match.group(3))
                    qz = float(match.group(4))
                    return {'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz}
        
        return None
    
    def start_continuous_mode(self, sample_delay=0.05):
        """Run in continuous mode, reading quaternion data indefinitely"""
        self.running = True
        samples_count = 0
        error_count = 0
        last_status_time = time.time()
        
        # Setup signal handler for graceful exit with Ctrl+C
        def signal_handler(sig, frame):
            print("\nStopping continuous mode...")
            self.running = False
        
        signal.signal(signal.SIGINT, signal_handler)
        
        print("Starting continuous quaternion reading mode. Press Ctrl+C to exit.")
        
        try:
            while self.running:
                quat = self.get_quaternion()
                current_time = time.time()
                
                if quat:
                    samples_count += 1
                    # Print status update every 5 seconds
                    if current_time - last_status_time > 5:
                        print(f"Running... Total samples: {samples_count}, Errors: {error_count}")
                        print(f"Latest: qw={quat['qw']:.4f}, qx={quat['qx']:.4f}, qy={quat['qy']:.4f}, qz={quat['qz']:.4f}")
                        last_status_time = current_time
                else:
                    error_count += 1
                    if error_count % 10 == 0:  # Only print every 10th error
                        print(f"Failed to get reading (errors: {error_count})")
                
                time.sleep(sample_delay)
                
        except Exception as e:
            print(f"Error in continuous mode: {e}")
        finally:
            print(f"Continuous mode stopped. Total samples: {samples_count}, Errors: {error_count}")
        
    def get_quaternion_stream(self, num_samples=10, delay=0.05):
        """Get multiple quaternion readings"""
        print(f"Reading {num_samples} quaternion samples...")
        readings = []
        
        for i in range(num_samples):
            quat = self.get_quaternion()
            if quat:
                readings.append(quat)
                print(f"Sample {i+1}/{num_samples}: {quat}")
            else:
                print(f"Sample {i+1}/{num_samples}: Failed to get reading")
            time.sleep(delay)
            
        return readings
        
    def close(self):
        """Close the serial connection"""
        if self.ser:
            self.ser.close()
            print("Connection closed")

# Example usage
if __name__ == "__main__":
    # Use the correct port for your Arduino
    port = '/dev/ttyACM0'  # Linux
    # port = 'COM3'        # Windows
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    bridge = BNO085Bridge(port=port)
    
    try:
        if bridge.connect():
            print("\nTesting BNO085 bridge:")
            
            # Scan I2C bus
            bridge.scan_i2c()
            
            # Initialize BNO085
            if bridge.initialize_bno085():
                print("\nBNO085 initialized successfully")
                
                # Enable rotation vector
                bridge.enable_rotation_vector()
                
                # Wait for sensor to stabilize
                print("\nWaiting for sensor to stabilize...")
                time.sleep(1)
                
                # Start continuous mode
                bridge.start_continuous_mode()
            else:
                print("Failed to initialize BNO085")
                # Try to reinitialize a couple of times
                for i in range(2):
                    print(f"\nRetrying initialization (attempt {i+1}/2)...")
                    time.sleep(1)
                    if bridge.initialize_bno085():
                        print("\nBNO085 initialized successfully on retry")
                        bridge.enable_rotation_vector()
                        print("\nWaiting for sensor to stabilize...")
                        time.sleep(1)
                        bridge.start_continuous_mode()
                        break
                    time.sleep(2)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bridge.close()