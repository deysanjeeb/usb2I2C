#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Configuration
#define BAUD_RATE 115200
#define BNO085_ADDRESS 0x4A  // Default I2C address for BNO085
#define MAX_BUFFER_LENGTH 256
#define COMMAND_TIMEOUT 1000  // Timeout in milliseconds

// Command definitions for our protocol
#define CMD_SCAN 'S'           // Scan I2C bus
#define CMD_INIT_BNO 'I'       // Initialize BNO085
#define CMD_GET_QUAT 'Q'       // Get quaternion data
#define CMD_ENABLE_ROTATION 'E' // Enable rotation vector reports
#define CMD_RESET 'R'          // Reset the BNO085 and communications

// Buffer for serial communication
uint8_t buffer[MAX_BUFFER_LENGTH];

// Create the BNO085 sensor object
Adafruit_BNO08x* bno08x = NULL;

// BNO085 Reports
sh2_SensorValue_t sensorValue;

// Flag to track if the sensor has been initialized
bool sensorInitialized = false;

// Timestamp for detecting serial reconnection
unsigned long lastSerialActivity = 0;
unsigned long serialTimeoutThreshold = 5000; // 5 seconds

// Function prototypes
void completeReset();

void getQuaternion() {
  // Check if sensor is initialized
  if (!bno08x || !sensorInitialized) {
    Serial.println("BNO085 not initialized. Run 'I' command first.");
    return;
  }
  
  // Check if BNO085 was reset
  if (bno08x->wasReset()) {
    Serial.println("BNO085 was reset - reinitializing");
    enableRotationVector();
    delay(100);
  }
  
  // Try multiple times to get data
  byte attempts = 5;
  bool dataReceived = false;
  
  while (!dataReceived && attempts > 0) {
    if (bno08x->getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        // Convert quaternion data to floating point
        float qw = sensorValue.un.rotationVector.real;
        float qx = sensorValue.un.rotationVector.i;
        float qy = sensorValue.un.rotationVector.j;
        float qz = sensorValue.un.rotationVector.k;
        float accuracy = sensorValue.un.rotationVector.accuracy;
        
        // Print quaternion data
        Serial.print("Quaternion: qw=");
        Serial.print(qw, 4);
        Serial.print(" qx=");
        Serial.print(qx, 4);
        Serial.print(" qy=");
        Serial.print(qy, 4);
        Serial.print(" qz=");
        Serial.print(qz, 4);
        Serial.print(" accuracy=");
        Serial.println(accuracy);
        
        dataReceived = true;
      } else {
        // Try again since we didn't get quaternion data
        attempts--;
        delay(5);
      }
    } else {
      attempts--;
      delay(20);
    }
  }
  
  if (!dataReceived) {
    Serial.println("No quaternion data available");
  }
}

void scanI2CBus() {
  Serial.println("Scanning I2C bus...");
  byte count = 0;
  
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (i < 16) {
        Serial.print("0");
      }
      Serial.print(i, HEX);
      Serial.print(" (");
      
      // Print if it's a known device
      if (i == BNO085_ADDRESS) {
        Serial.print("BNO085");
      } else {
        Serial.print("unknown device");
      }
      
      Serial.println(")");
      count++;
    }
  }
  
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s)");
}

bool initializeBNO085() {
  // Clean up any existing object
  if (bno08x) {
    delete bno08x;
    bno08x = NULL;
  }
  
  // Reset I2C bus
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(100000); // Start with lower speed for stability
  delay(50);
  
  // Create a new BNO085 object
  bno08x = new Adafruit_BNO08x();
  if (!bno08x) {
    Serial.println("Failed to create BNO085 object");
    return false;
  }
  
  // Initialize with I2C
  Serial.println("Attempting to initialize BNO085...");
  if (!bno08x->begin_I2C(BNO085_ADDRESS)) {
    Serial.println("Error: Failed to initialize BNO085. Check connections.");
    delete bno08x;
    bno08x = NULL;
    sensorInitialized = false;
    return false;
  }
  
  Serial.println("BNO085 initialized successfully");
  sensorInitialized = true;
  
  // Give the sensor a moment to stabilize
  delay(100);
  
  return true;
}

void enableRotationVector() {
  if (!bno08x || !sensorInitialized) {
    Serial.println("BNO085 not initialized. Run 'I' command first.");
    return;
  }
  
  Serial.println("Enabling rotation vector reports...");
  
  // Set report interval (10ms = 100Hz)
  if (!bno08x->enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable rotation vector");
    return;
  }
  
  Serial.println("Rotation vector enabled");
  delay(100);
}

void completeReset() {
  Serial.println("Performing complete reset of all systems");
  
  // Clean up sensor object
  if (bno08x) {
    delete bno08x;
    bno08x = NULL;
  }
  
  sensorInitialized = false;
  
  // Reset I2C
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(100000); // Start with conservative speed
  
  Serial.println("I2C bus reset complete");
  
  // Rescan
  scanI2CBus();
}

void checkSerialTimeout() {
  // If we haven't seen serial activity for a while and then get new activity,
  // assume we had a reconnection and reset everything
  if (millis() - lastSerialActivity > serialTimeoutThreshold && Serial.available()) {
    Serial.println("Serial reconnection detected - performing complete reset");
    completeReset();
  }
  
  if (Serial.available()) {
    lastSerialActivity = millis();
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  
  // Don't wait for serial port in setup, as this could block indefinitely
  // if the Arduino is powered but not connected to a computer
  
  // Initialize I2C as master
  Wire.begin();
  Wire.setClock(100000); // Start with lower speed for reliability
  
  // Record initial serial activity time
  lastSerialActivity = millis();
  
  delay(1000); // Give everything time to stabilize
  
  Serial.println("\n\nArduino I2C Bridge for BNO085 initialized");
  Serial.println("Commands:");
  Serial.println("  S - Scan I2C bus");
  Serial.println("  I - Initialize BNO085");
  Serial.println("  E - Enable rotation vector");
  Serial.println("  Q - Get quaternion data");
  Serial.println("  R - Reset BNO085 and I2C");
  
  // Scan I2C bus at startup
  scanI2CBus();
}

void loop() {
  // Check for serial timeout/reconnection
  checkSerialTimeout();
  
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Update last serial activity time
    lastSerialActivity = millis();
    
    switch (cmd) {
      case CMD_SCAN:
        scanI2CBus();
        break;
        
      case CMD_INIT_BNO:
        initializeBNO085();
        break;
        
      case CMD_ENABLE_ROTATION:
        enableRotationVector();
        break;
        
      case CMD_GET_QUAT:
        getQuaternion();
        break;
        
      case CMD_RESET:
        completeReset();
        break;
        
      default:
        // Ignore unrecognized commands
        break;
    }
  }
  
  // Process any pending I2C data to keep the connection active
  if (bno08x && sensorInitialized) {
    bno08x->getSensorEvent(&sensorValue);
  }
}
