#include "Wire.h"

#define MP6546_ADDRS {0x07, 0x08, 0x09}  // I²C addresses
#define POLE_PAIRS    4                   // Motor pole pairs
#define VIN           12.0                 // Driver supply voltage (V)
#define BIAS_OFFSET   500                 // Hall sensor DC offset (mV)
#define BIAS_SCALE    6.445               // Scaling factor from datasheet

byte addr = 0x08;

void scan_i2c(){
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delay(50);
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      addr = address;
      nDevices++;
    } else if (error != 2) {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  }
}

void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint16_t data) {
  Wire.beginTransmission(deviceAddress);  // Start communication with device
  Wire.write(registerAddress);            // Send register address
  Wire.write((data >> 8) & 0xFF);  // data MSB first
  Wire.write(data & 0xFF);         // data LSB next
  Wire.endTransmission();                 // End transmission
}

void setConfiguration() {
  uint16_t config = 0xF6F3;                   // 1 1 1 10 1 1 0 1 11 1 0 011
  writeRegister(addr, 0x00, config);
}

// Ha max = 1.54 
// Ha min = 0.410
// Hb max = 0.420
// Hb min = 1.54
void configureHallSensors(){
  uint16_t hall_data = 0x4A4A;                  
  writeRegister(addr,0x01,hall_data);          // set ADC for hall sensor
}

void configureThetaAndPolePairs() {
  uint16_t reg02 = 0x0004;        // 000 (theta bias)  4 (pole pair) 
  writeRegister(addr, 0x02, reg02);
}

void enterNormalMode(){
  uint16_t data = 0xFF;
  writeRegister(addr, 0x0F, data);
}

void enterStandbyMode(){
  uint16_t data = 0xFF;
  writeRegister(addr, 0x10, data);
}

void move_angle(uint16_t angle){
  if (angle > 4095) angle = 4095;  // Clamp angle
  writeRegister(addr, 0x07, angle);
}

uint16_t readRegister( uint8_t registerAddress) {
  Wire.beginTransmission(addr);
  Wire.write(registerAddress);  // Specify register to read
  if (Wire.endTransmission(false) != 0) { // repeated start
    Serial.println("Transmission error");
    return 0xFFFF; // error code
  }

  Wire.requestFrom(addr, (uint8_t)2);  // Request 2 bytes
  if (Wire.available() < 2) {
    Serial.println("Read error");
    return 0xFFFF;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (msb << 8) | lsb;
}

void setup() {
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);
  delay(1000);
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  delay(1000);
  enterStandbyMode();
  delay(1000);
  setConfiguration();
  delay(1000);
  configureHallSensors();
  delay(1000);
  configureThetaAndPolePairs();
  delay(1000);
  writeRegister(addr, 0x08, 0x0000);  // UD
  delay(100);
  writeRegister(addr, 0x09, 0x01D1);  // UQ ≈ 465
  delay(100);
  enterNormalMode();
  delay(1000);
  writeRegister(addr,0x07,0x100);         // set A phase register to 100%
  writeRegister(addr,0x08,0x100);        // set B phase register to 100%
  writeRegister(addr,0x09,0x100);        // set C phase register to 100%
  Serial.println("Configuration done");
  Serial.println("Enter a duty cycle");
  
}

void loop() {
  if (Serial.available() > 0) {
    uint16_t pwm = Serial.parseInt();  // reads the next integer
    pwm &= 0x1FF;
    //pwm ^= 0x0FF;
    writeRegister(addr,0x07,pwm);         // set A phase register
    writeRegister(addr,0x08,pwm);        //set B phase register
    writeRegister(addr,0x09,pwm);        //set C phase register
    Serial.print("duty cycle written as ");Serial.println(pwm);
    Serial.println("Enter a new duty cycle if want to change ");
  }
  uint16_t pwm = Serial.parseInt();
  delay(1000);
  uint16_t data = readRegister(0x07);
  Serial.print("0x07: "); Serial.println(data);

   data = readRegister(0x08);
  Serial.print("0x08: "); Serial.println(data);

   data = readRegister(0x09);
  Serial.print("0x09: "); Serial.println(data);

   data = readRegister(0x00);
  Serial.print("0x00: "); Serial.println(data);

  data = readRegister(0x01);
  Serial.print("0x01: "); Serial.println(data);

   data = readRegister(0x02);
  Serial.print("0x02: "); Serial.println(data);
}
