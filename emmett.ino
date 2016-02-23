/**                                __    __
 *    ____   _____   _____   _____/  |__/  |_
 *  _/ __ \ /     \ /     \_/ __ \   __\   __\
 *  \  ___/|  Y Y  \  Y Y  \  ___/|  |  |  |
 *   \_____>__|_|  /__|_|  /\_____>__|  |__|
 *               \/      \/ 
 * openEmmett
 * DIY one-wheel, self-balancing, electric skateboard. AKA open-source hoverboard.
 * https://github.com/doublejosh/emmett
 * v0.2.2
 *
 * Arduinos tested: Leonardo, Nano, Pro Mini
 * Accelerometer: ADXL345
 * DAC: MCP4725
 */

// I2C.
#include <Wire.h>

// Main adjustments.
const float minPowerAngle    = 1;
const float maxPowerAngle    = 5;
const float powerDelayAngle  = 2;
const float powerDelay       = 6; // @todo Switch to time-based.
const float powerOffDelay    = 10;
const float maxTiltAngle     = 7;
const float minThrottleVolts = 2.5;
const float maxThrottleVolts = 3.0;
const float angleAdjust      = 1.23;
const float tiltAdjust       = 1.2;
const boolean useAverage     = false;
const boolean rotateAcc      = true; // Accelerometer rotate 90 degrees.
const boolean debugging      = true;
const boolean debugOcillate  = false; // Basic throttle ocillation.
const int     readDelay      = 1; // Never zero.

// TEMPORARY.
int ocillate = 0;
boolean ocillateDirection = true;

// Pin configurations.
const int   THROTTLE    = 3;  // Throttle control.
const int   REVERSE     = 6;  // Reverse relay.
//const int   LIMITER     = 10; // Power limiter.
//const int   ACC_CS      = 2;  // Chip select: accelerometer.
//const int   DAC_CS      = 4;  // Chip select: digital to analog.
// NOTE: SDA = A4.
// NOTE: SCL = A5.
const int   LED_RED_1   = 4;  // Analog output to RGB LED strip #1 (front).
const int   LED_GREEN_1 = 5;
const int   LED_BLUE_1  = 7;
const int   LED_RED_2   = 8;  // Analog output to RGB LED strip #2 (back).  
const int   LED_GREEN_2 = 9;
const int   LED_BLUE_2  = 10;

const int   BOARD_VOLTAGE = 5; // Arduino voltage.
const int   DAC_MAX       = 4095; // Signal to voltage boundary.

// Range boundaries for angle to DAC.
const int minSignal = mapFloat(minThrottleVolts, 0, BOARD_VOLTAGE, 0, DAC_MAX);
const int maxSignal = mapFloat(maxThrottleVolts, 0, BOARD_VOLTAGE, 0, DAC_MAX);

// Internal globals.
float angles[5]; // Rolling average.
int readIndex = 0; // Floating average sensor angle.
int rideStatus = 0; // Current device state.
int powerDelayCycles = 0; // Power start counter.
int powerOffCycles = 0; // Power off counter.
float data[2]; // Processed sensor data for application consumption.
unsigned int rgbColor[3] = {255, 0, 0};
//unsigned long previousMillis = 0; // Store time.
//const long LONG_RESET = 2147483000; // Careful with variable value.

// Buffer.
uint8_t values[6]; // I2C.

// I2C device addresses.
#define I2C_ACC_ADDR (0x53)  // Alternatives: 0x53, 0xA6, 0xA7.
#define I2C_DAC_ADDR (0x60)

// ADXL345 registers.
const char DATAX0 = 0x32; //X-Axis Data 0
const char DATAX1 = 0x33; //X-Axis Data 1
const char DATAY0 = 0x34; //Y-Axis Data 0
const char DATAY1 = 0x35; //Y-Axis Data 1
const char DATAZ0 = 0x36; //Z-Axis Data 0
const char DATAZ1 = 0x37; //Z-Axis Data 1
// Supposed alternate data addresses.
//const char DATAX0 = 0xB2; //X-Axis Data 0
//const char DATAX1 = 0xB3; //X-Axis Data 1
//const char DATAY0 = 0xB4; //Y-Axis Data 0
//const char DATAY1 = 0xB5; //Y-Axis Data 1
//const char DATAZ0 = 0xB6; //Z-Axis Data 0
//const char DATAZ1 = 0xB7; //Z-Axis Data 1

// Statup.
void setup() {
  // Initiate i2c communication.
  Wire.begin();

  // Set the chip select for output. Required for SPI.
  // pinMode(ACC_CS, OUTPUT);
  // digitalWrite(ACC_CS, HIGH);

  if (debugging) {
    Serial.begin(9600); 
    Serial.print("minSignal: ");
    Serial.println(minSignal);
    Serial.print("maxSignal: ");
    Serial.println(maxSignal);
  }

  // Setup motor limiter.
  //pinMode(LIMITER, INPUT);
  // Set the motor speed throttle via PWM.
  pinMode(THROTTLE, OUTPUT);
  // Set the reverse relay for output.
  pinMode(REVERSE, OUTPUT);
  digitalWrite(REVERSE, LOW);

  // This code is for SPI, but setting range can be necessary with a new ADXL345.
  // Accelerometer range to the DATA_FORMAT register: +/- 4G range = 0x01.
  writeData(I2C_ACC_ADDR, 0x31, 11);
  // Accelerometer measurement mode to the POWER_CTL register: 0x08.
  //writeData(I2C_ACC_ADDR, 0x2D, 0x08);

  // Set x, y, z offsets.
  writeData(I2C_ACC_ADDR, 0x2D, 0);
  writeData(I2C_ACC_ADDR, 0x2D, 16);
  writeData(I2C_ACC_ADDR, 0x2D, 8);

  // LEDs.
  // pinMode(LED_RED, OUTPUT);
  // pinMode(LED_GREEN, OUTPUT);
  // pinMode(LED_BLUE, OUTPUT);
}

// Continue.
void loop() {
//  unsigned long currentMillis = millis();
//  
//  // Time difference.
//  if (currentMillis - previousMillis >= interval) {
//    // Save time.
//    previousMillis = currentMillis;
//
//  }
//  if (previousMillis >= LONG_RESET) {
//    previousMillis = 0;
//  }

  // For calibration testing.
  if (debugOcillate) {
    if (ocillate >= 255) {
      ocillateDirection = false;
    }
    else if (ocillate <= 0) {
      ocillateDirection = true;
    }

    if (ocillateDirection) {
      ocillate++;
    }
    else {
      ocillate--;
    }
    
    Serial.println(ocillate);
    analogWrite(THROTTLE, ocillate);
  }
  else {
    powerMotor(manageRide(findAngles(data)));
  }

  delay(readDelay);
}


/**
 * Use sensor data for overall UX.
 *
 * 0 = resting
 * 1 = on
 * ...more to come.
 */
float manageRide(float *data) {
  float angle = data[0];
  float angleSize = abs(angle);

  // Shut down when tilted out-of-bounds.
  if (abs(data[1]) > maxTiltAngle) {
    if (debugging) {
      Serial.print("TILT! -- ");
    }
    rideStatus = 0;
    // @todo Duplicated.
    powerOffCycles = 0;
    powerDelayCycles = 0;
    return 0;
  }

  // Already on.
  if (rideStatus == 1) {
    // Within acceleration bounds.
    if (angleSize > minPowerAngle && angleSize < maxPowerAngle) {
      return angle;
    }
    // Level, glide.
    else if (angleSize < minPowerAngle) {
      return 0;
    }
    // Non-riding position.
    else {
      // Allow turning off for later power delay.
      if (powerOffCycles >= (powerOffDelay / readDelay)) {
        // Begin off state.
        rideStatus = 0;
        powerOffCycles = 0;
        powerDelayCycles = 0;
      }
      powerOffCycles++;
      return 0;
    }
  }
  else if (angleSize < powerDelayAngle) {
    // Off, but level enough to start...
    if (debugging) {
      Serial.print(" --- [ ");
      Serial.print(powerDelayCycles);
      Serial.print(" | ");
      Serial.print(powerDelay);
      Serial.print(" | ");
      Serial.print(readDelay);
      Serial.print(" ]");
    }
    // Allow crossing power delay.
    if (powerDelayCycles >= (powerDelay / readDelay)) {
      // Riding start state.
      rideStatus = 1;
      powerDelayCycles = 0;
      powerOffCycles = 0;
      return angle;
    }
    powerDelayCycles++;
    return 0;
  }
  else {
    // Non-use state.
    powerDelayCycles = 0;
    return 0;
  }
}

/**
 * Send power to motor. Zero = off, value = angle.
 */
void powerMotor(float angle) {
  boolean motorDirection = true;
  
  // Avoid computation when ride is off.
  if (angle != 0) {

    // Allow adjusting power.
    //float powerLimit = 1.0; //map(analogRead(LIMITER), 0, 1063, 0, 1.0);
    
    // Send angle as analog output voltage.
    int signal = mapFloat(abs(angle), minPowerAngle, maxPowerAngle, minSignal, maxSignal);
    analogSignal(I2C_DAC_ADDR, signal);

    // Send angle as PWM output voltage.
    float voltage = mapFloat(abs(angle), minPowerAngle, maxPowerAngle, minThrottleVolts, maxThrottleVolts);
    int pwm = 255 * (voltage / BOARD_VOLTAGE);
    analogWrite(THROTTLE, pwm);

    // Set reverse.
    if (angle > 1) {
      digitalWrite(REVERSE, HIGH);
      motorDirection = false;
    }
    else {
      digitalWrite(REVERSE, LOW);
    }

    //displayColor(motorDirection);

    // Debugging.
    if (debugging) {
      Serial.print(" Volts: ");
      Serial.print(voltage);
      Serial.print(" PWM: ");
      Serial.print(pwm);
      Serial.print(" DAC: ");
      Serial.print(signal);
      Serial.print(" -- angle: ");
      Serial.print(angle);
      Serial.print(" -- status: ");
      Serial.println(rideStatus);
    }
  }
  else {
    // Ouput PWM off.
    analogWrite(THROTTLE, LOW);

    if (debugging) {
      Serial.print("OFF");
      Serial.print(" ----- status: ");
      Serial.println(rideStatus);
    }
  }
}

/**
 * Determine orientation of device.
 */
float* findAngles(float *data) {
  int x, y, z;
  float newAngle = 0;

  // Read x,y and z from accelerometerto the values[] buffer.
  // The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits).
  // To get the full value, two bytes must be combined for each axis.
  readData(I2C_ACC_ADDR, DATAX0, 6, values);
  x = (((int)values[1]) << 8) | values[0];
  y = (((int)values[3]) << 8) | values[2];
  z = (((int)values[5]) << 8) | values[4];

  // Calculate an angle between -360 and 360.
  if (rotateAcc) {
    //newAngle = (atan2(x , z) * 57.3) + angleAdjust;
    newAngle = (atan2(x, z) * 180.0f / M_PI) + angleAdjust;
    
    // atan2(x, z) * 57.3
    // atan2(x, z) * 180.0f / M_PI
    // atan2((- x) , sqrt(y * y + z * z)) * 57.3
  }
  else {
    //newAngle = (atan2(y , z) * 57.3) + angleAdjust;
    newAngle = (atan2(y, z) * 180.0f / M_PI) + angleAdjust;
  }

  // Rolling average option.
  if (useAverage) {
    // Overwrite new average buffer value.
    angles[readIndex] = newAngle;
    data[0] = (angles[0] + angles[1] + angles[2] + angles[3] + angles[4]) / 5 + angleAdjust;
    readIndex++;
    if (readIndex >= 4) {
      readIndex = 0;
    }
  }
  else {
    data[0] = newAngle;
  }

  if (rotateAcc) {
    // Calculate tilt for out-of-bounds/crash sensing.
    //data[1] = atan2(y, z) * 180.0f / M_PI;
    data[1] = (atan2(y , z) * 57.3) + tiltAdjust;
  }
  else {
    data[1] = (atan2(x , z) * 57.3) + tiltAdjust;
  }

  // Filter data (alternate sensor filter option).
  //  xf = FILTER_TIME * xf + (1.0 - FILTER_TIME) * x;
  //  yf = FILTER_TIME * yf + (1.0 - FILTER_TIME) * y;

  if (debugging) {
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.print(" Pitch: ");
    Serial.print(data[0]);
    Serial.print(" - Roll: ");
    Serial.print(data[1]); 
    Serial.print(" - ");
  }

  return data;
}

/**
 * Control lights.
 */
void displayColor(int motorDirection) {
  if (rideStatus == 0) {
    // @todo Rainbow. 
    analogWrite(LED_RED_1,   LOW);
    analogWrite(LED_GREEN_1, LOW);
    analogWrite(LED_BLUE_1,  LOW);
    analogWrite(LED_RED_2,   LOW);
    analogWrite(LED_GREEN_2, LOW);
    analogWrite(LED_BLUE_2,  LOW);
  }
  else {
    // Headlights and tail lights.
    if (motorDirection) {
      analogWrite(LED_RED_1,   HIGH);
      analogWrite(LED_GREEN_1, LOW);
      analogWrite(LED_BLUE_1,  LOW);

      analogWrite(LED_RED_2,   HIGH);
      analogWrite(LED_GREEN_2, HIGH);
      analogWrite(LED_BLUE_2,  HIGH);
    }
    else  {
      analogWrite(LED_RED_1,   HIGH);
      analogWrite(LED_GREEN_1, HIGH);
      analogWrite(LED_BLUE_1,  HIGH);

      analogWrite(LED_RED_2,   HIGH);
      analogWrite(LED_GREEN_2, LOW);
      analogWrite(LED_BLUE_2,  LOW);
    }
  }
}

/**
 * Ouput voltage with the DAC.
 */
void analogSignal(int DEVICE, int value) {
  Wire.beginTransmission(DEVICE);
  // Update the DAC.
  Wire.write(64);
  // 8 most significant bits...
  Wire.write(value >> 4);
  // 4 least significant bits... 
  Wire.write((value & 15) << 4);
  Wire.endTransmission();
}

/**
 * Map floats.
 */
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/**
 * Write to communication register.
 *
 * char registerAddress - The register to write a value to
 * char value           - The value to be written to the specified register.
 */
void writeData(int DEVICE, char address, char value) {
  Wire.beginTransmission(DEVICE);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * Read data registers into values buffer.
 */
void readData(int DEVICE, char address, int numBytes, unsigned char * values) {
  Wire.beginTransmission(DEVICE);
  Wire.write(address);
  Wire.endTransmission();
  Wire.beginTransmission(DEVICE);
  Wire.requestFrom(DEVICE, numBytes);
  
  int i = 0;
  while (Wire.available()) {
    values[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}

