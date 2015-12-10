/**
 * Motion controlled electric skateboard.
 *
 * Arduino: Leonaro
 * Accelerometer: ADXL345
 */

// Add the SPI library so we can communicate with the ADXL345 sensor.
#include <SPI.h>

// Assign the Chip Select signal pin.
int CS = 8;

// Some of the ADXL345 registers.
char POWER_CTL = 0x2D;
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1
//char DATAX0 = 0xB2;	//X-Axis Data 0
//char DATAX1 = 0xB3;	//X-Axis Data 1
//char DATAY0 = 0xB4;	//Y-Axis Data 0
//char DATAY1 = 0xB5;	//Y-Axis Data 1
//char DATAZ0 = 0xB6;	//Z-Axis Data 0
//char DATAZ1 = 0xB7;	//Z-Axis Data 1

// Buffer for ADXL345 register values.
unsigned char values[10];
// Hold the x,y and z axis accelerometer values.
int x,y,z;

void setup() { 
  // Initiate and configure the SPI communication.
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  // Create a serial connection for debugging.
  Serial.begin(9600);
  
  // Set the Chip Select for output.
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  
  // Put ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);

  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);
}

void loop() {
  
  // Print the results to the terminal.
  Serial.println( findAngle() );

  delay(20);
}


/**
 * Determine orientation of device.
 */
float findAngle() {
  // Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values
  // from the ADXL345. The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  // The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits).
  // To get the full value, two bytes must be combined for each axis.
  // The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  // The Y value is stored in values[2] and values[3].
  //y = ((int)values[3]<<8)|(int)values[2];
  // The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];
  
  // Calculate angle will be between -360 and 360 degrees.
  float angle = atan2(x, z) * 180.0f / M_PI; 
  return angle; 
}


/**
 * Write a value to a register on the ADXL345.
 *
 * char registerAddress - The register to write a value to
 * char value           - The value to be written to the specified register.
 */
void writeRegister(char registerAddress, char value) {
  // Signal beginning of an SPI packet.
  digitalWrite(CS, LOW);
  // Transfer the register address, then value.
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  // Signal end of an SPI packet.
  digitalWrite(CS, HIGH);
}


/**
 * Read registers starting from address, store values in buffer.
 * 
 * char registerAddress - Register addresses to start the read sequence.
 * int  numBytes        - Number of registers to read.
 * char * values        - Pointer to a buffer where operation results should be stored.
 */
void readRegister(char registerAddress, int numBytes, unsigned char * values) {
  // Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  // If we're doing a multi-byte read, bit 6 needs to be set as well.
  if (numBytes > 1) address = address | 0x40;
  
  // Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  // Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  // Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for (int i=0; i<numBytes; i++) {
    values[i] = SPI.transfer(0x00);
  }
  // Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

