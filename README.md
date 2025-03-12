# BNO085 Arduino USB-to-I2C Bridge

This project provides a Python interface to communicate with a BNO085 IMU sensor via an Arduino acting as a USB-to-I2C bridge. The Arduino receives commands over serial and relays them to the BNO085 sensor via I2C, allowing a computer to interact with the sensor as if it were directly connected.

## Features
- Connects to an Arduino via USB.
- Sends commands to interact with the BNO085 sensor.
- Reads quaternion data from the IMU.
- Supports continuous data streaming.
- Graceful shutdown with Ctrl+C.

## Requirements
### Hardware
- **Arduino Uno** (or similar board with I2C capability)
- **BNO085 IMU Sensor**
- **Computer running Linux or Windows**

### Software
- **Python 3.x**
- **Required Python libraries:**
  - `pyserial`

Install dependencies using:
```sh
pip install pyserial
```

## Setup
1. **Connect the BNO085** sensor to the Arduino via I2C.
2. **Upload the appropriate firmware** to the Arduino (ensure it is programmed to forward I2C commands from serial input).
3. **Connect the Arduino** to your computer via USB.
4. **Find the serial port** of your Arduino:
   ```sh
   ls /dev/ttyACM*
   ```
   or on Windows, check Device Manager.

## Usage
Run the Python script with:
```sh
python3 bno085_bridge.py /dev/ttyACM0
```
For Windows, use:
```sh
python bno085_bridge.py COM3
```

### Commands
The script sends predefined commands to the Arduino:
- `S` - Scan for I2C devices.
- `I` - Initialize the BNO085.
- `E` - Enable rotation vector mode.
- `Q` - Get quaternion data.

### Continuous Mode
To read quaternion data continuously, press Ctrl+C to stop execution safely.

## Troubleshooting
- **"Error connecting to Arduino"**: Ensure the correct port is specified.
- **"Failed to initialize BNO085"**: Try restarting the sensor.
- **Unexpected values**: Allow some time for sensor stabilization.

