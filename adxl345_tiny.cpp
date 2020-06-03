/**************************************************************************/
/*!
    @file     adxl345_tiny.cpp
    @author   Steffen Schmelter (IDiAL)
    @license  Based on: https://github.com/adafruit/Adafruit_ADXL345/
    @section  HISTORY
    v1.0  - First release
*/
/**************************************************************************/

// SENSOR DATASHEET
// EN: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "adxl345_tiny.h"


/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino Wire library
*/
/**************************************************************************/
inline uint8_t ADXL345::i2cread(void) {
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino Wire library
*/
/**************************************************************************/
inline void ADXL345::i2cwrite(uint8_t x) {
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void ADXL345::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission((uint8_t)_i2caddr);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value));
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t ADXL345::readRegister(uint8_t reg) {
  Wire.beginTransmission((uint8_t)_i2caddr);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2caddr, 1);
  return (i2cread());
}

/**************************************************************************/
/*!
    @brief  Reads value of specified Bit from the specified register
*/
/**************************************************************************/
bool ADXL345::readRegisterBit(uint8_t reg, uint8_t bitToRead) {
  /* Read the data format register to preserve bits */
  uint8_t register_val = readRegister(reg);

  register_val &= 0b1 << bitToRead; // Mask out only the Bit of specific Register
  
  return (register_val != 0); // Returns true when masked out Bit is 1
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16_t ADXL345::read16(uint8_t reg) {
  Wire.beginTransmission((uint8_t)_i2caddr);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2caddr, 2);
  return (uint16_t)(i2cread() | (i2cread() << 8));  
}

/**************************************************************************/
/*! 
    @brief  Reads the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t ADXL345::getDeviceID(void) {
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent X axis value (respectively same for Y and Z)
*/
/**************************************************************************/
int16_t ADXL345::getRawX(void) {
  return read16(ADXL345_REG_DATAX0);
}

int16_t ADXL345::getRawY(void) {
  return read16(ADXL345_REG_DATAY0);
}

int16_t ADXL345::getRawZ(void) {
  /*
  uint8_t lsbs = read16(ADXL345_REG_DATAZ0);
  uint8_t msbs = readRegister(ADXL345_REG_DATAZ1);

  Serial.print("m, l: ");
  Serial.print(msbs);
  Serial.print(", ");
  Serial.print(lsbs);
  Serial.println("");
  */
  
  return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class
*/
/**************************************************************************/
ADXL345::ADXL345(int32_t sensorID) {
  _sensorID = sensorID;
  _range = ADXL345_RANGE_2_G;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool ADXL345::begin(uint8_t i2caddr, bool resetToDefault) {
  _i2caddr = i2caddr;

  Wire.begin();

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5)
  {
    /* No ADXL345 detected ... return false */
    return false;
  }

  if(resetToDefault) {
      resetToDefaultValues();
  }
  
  // Enable measurements. Set Power on via Measure Bit D3=1 on Address 0x2D
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);  
    
  return true;
}

/**************************************************************************/
/*!
    @brief  Resets Registers to their initial default values
*/
/**************************************************************************/
void ADXL345::resetToDefaultValues(void) 
{
    writeRegister(ADXL345_REG_THRESH_TAP,     0x00);
    writeRegister(ADXL345_REG_OFSX,           0x00);
    writeRegister(ADXL345_REG_OFSY,           0x00);
    writeRegister(ADXL345_REG_OFSZ,           0x00);
    writeRegister(ADXL345_REG_DUR,            0x00);
    writeRegister(ADXL345_REG_LATENT,         0x00);
    writeRegister(ADXL345_REG_WINDOW,         0x00);
    writeRegister(ADXL345_REG_THRESH_ACT,     0x00);  
    writeRegister(ADXL345_REG_THRESH_INACT,   0x00);
    writeRegister(ADXL345_REG_TIME_INACT,     0x00);
    writeRegister(ADXL345_REG_ACT_INACT_CTL,  0x00);
    writeRegister(ADXL345_REG_THRESH_FF,      0x00);
    writeRegister(ADXL345_REG_TIME_FF,        0x00);
    writeRegister(ADXL345_REG_TAP_AXES,       0x00);
    writeRegister(ADXL345_REG_ACT_TAP_STATUS, 0x00);
    writeRegister(ADXL345_REG_BW_RATE,        0b00001010);
    writeRegister(ADXL345_REG_POWER_CTL,      0x00);
    writeRegister(ADXL345_REG_INT_ENABLE,     0x00);
    writeRegister(ADXL345_REG_INT_MAP,        0x00);
    writeRegister(ADXL345_REG_INT_SOURCE,     0b00000010);
    writeRegister(ADXL345_REG_DATA_FORMAT,    0x00);
    writeRegister(ADXL345_REG_FIFO_CTL,       0x00);
    writeRegister(ADXL345_REG_FIFO_STATUS,    0x00);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void ADXL345::setRange(range_t range)
{
  /* Read the data format register to preserve bits */
  uint8_t format = readRegister(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;
  
  /* Make sure that the FULL-RES bit D3 is enabled for range scaling */
  format |= 0x08; //0000 1000
  
  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);
  
  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
*/
/**************************************************************************/
range_t ADXL345::getRange(void)
{
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void ADXL345::setDataRate(dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  writeRegister(ADXL345_REG_BW_RATE, dataRate);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t ADXL345::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool ADXL345::getRawAccelData(accel_data_t *data) {
  /* Clear the event */
  memset(data, 0, sizeof(accel_data_t));

  data->acceleration_x = getRawX();
  data->acceleration_y = getRawY();
  data->acceleration_z = getRawZ();
  
  return true;
}

bool ADXL345::getRawAccelDataBurst(accel_data_t *data) {
  Wire.beginTransmission((uint8_t)_i2caddr);
  i2cwrite(ADXL345_REG_DATAX0);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2caddr, 6);

  /* Clear the event */
  memset(data, 0, sizeof(accel_data_t));

  data->acceleration_x = (uint16_t)(i2cread() | (i2cread() << 8));
  data->acceleration_y = (uint16_t)(i2cread() | (i2cread() << 8));
  data->acceleration_z = (uint16_t)(i2cread() | (i2cread() << 8));

  return true;
}



void ADXL345::setAutoSleep(uint8_t acc_threshold, uint8_t time, bool autosleepEnabled)
{  
  /*
    Additional power can be saved if the ADXL345 automatically 
    switches to sleep mode during periods of inactivity. To 
    enable this feature, set the THRESH_INACT register (
    Address 0x25) and the TIME_INACT register (Address 0x26) 
    each to a value that signifies inactivity (the appropriate 
    value depends on the application)
  */

  // scale factor is 62.5 mg/LSB 
  // A value of 0 may result in undesirable behavior if the inactivity interrupt is enabled.
  writeRegister(ADXL345_REG_THRESH_INACT, acc_threshold);

  // scale factor is 1 sec/LSB
  writeRegister(ADXL345_REG_TIME_INACT, time);

  /* Read the data format register to preserve bits */
  uint8_t power_ctl = readRegister(ADXL345_REG_POWER_CTL);
  
  // Set the Link Bit (D5)
  power_ctl |=  0b00100000; // Set Link Bit to 1
 
  // Set the AUTO_SLEEP (D4)
  if(autosleepEnabled){
    power_ctl |=  0b00010000; // Set AUTO_SLEEP Bit to 1
  }else{
    power_ctl &= ~0b00010000; // Set AUTO_SLEEP Bit to 0
  }

  //Serial.println("AutoSleep PWR_CTL: " + String(power_ctl));

  writeRegister(ADXL345_REG_POWER_CTL, power_ctl);
}

void ADXL345::setSleepDataRate(sleepDataRate_t sleeptDataRate)
{
  /* Read the data format register to preserve bits */
  uint8_t power_ctl = readRegister(ADXL345_REG_POWER_CTL);

  /* Update the sleepdata rate (D0, D1) */

  power_ctl |= sleeptDataRate;
  
  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_POWER_CTL, power_ctl);
}

bool ADXL345::getSleepState(void)
{
  return readRegisterBit(ADXL345_REG_ACT_TAP_STATUS, 3);
}


void ADXL345::setActivityInterrupt(uint8_t acc_threshold, interruptPin_t intPin, bool interruptEnabled)
{
  // scale factor is 62.5 mg/LSB
  writeRegister(ADXL345_REG_THRESH_ACT, acc_threshold);

  /* Specifies Pin on which interrupt is fired */
  setInterruptPin(ADXL345_INTERRUPT_Activity, intPin);

  /* Enables or disables firing of interrupt on specified Pin */
  setInterrupt(ADXL345_INTERRUPT_Activity, interruptEnabled);
}

void ADXL345::setActivityDetectionAxis(detectMode_t detection_mode, bool ACT_X_enable, bool ACT_Y_enable, bool ACT_Z_enable)
{
  /* Read the data format register to preserve bits */
  uint8_t act_inact_ctl = readRegister(ADXL345_REG_ACT_INACT_CTL);

  // Defining the corresponding first 4 Bit of register
  byte act_register = 0x00;
  act_register = detection_mode << 7;
  act_register += ACT_X_enable << 6;
  act_register += ACT_Y_enable << 5;
  act_register += ACT_Z_enable << 4;

  act_inact_ctl &= 0b00001111; // Mask out to keep only last 4 Bits
  act_inact_ctl |= act_register; // Writing defined first 4 Bit while keeping last untouched

  //Serial.println("setActivityDetectionAxis: " + String(act_register));
  //Serial.println("setActivityDetectionAxisFULL: " + String(act_inact_ctl));

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_ACT_INACT_CTL, act_inact_ctl);
}

void ADXL345::setInActivityDetectionAxis(detectMode_t detection_mode, bool INACT_X_enable, bool INACT_Y_enable, bool INACT_Z_enable)
{
  /* Read the data format register to preserve bits */
  uint8_t act_inact_ctl = readRegister(ADXL345_REG_ACT_INACT_CTL);

  // Defining the corresponding last 4 Bit of register
  byte inact_register = 0x00;
  inact_register = detection_mode << 3;
  inact_register += INACT_X_enable << 2;
  inact_register += INACT_Y_enable << 1;
  inact_register += INACT_Z_enable;

  act_inact_ctl &= 0b11110000; // Mask out to keep only first 4 Bits
  act_inact_ctl |= inact_register; // Writing defined last 4 Bit while keeping last untouched

  //Serial.println("setInActivityDetectionAxis: " + String(inact_register));
  //Serial.println("setInActivityDetectionAxisFULL: " + String(act_inact_ctl));

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_ACT_INACT_CTL, act_inact_ctl);
}

void ADXL345::setInterrupt(interruptType_t interruptType, bool interruptEnabled)
{
  /* Read the data format register to preserve bits */
  uint8_t int_enable = readRegister(ADXL345_REG_INT_ENABLE);

  // Set the interruptType corresponding Bit to the corresponding value for specific int Pin to enable
  if(interruptEnabled){
    int_enable |= interruptType; // Set the corresponding Bit to 1
  }else{
    int_enable &= ~interruptType; // Set the corresponding Bit to 0
  }

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_INT_ENABLE, int_enable);
}

void ADXL345::setInterruptPin(interruptType_t interruptType, interruptPin_t intPin) 
{
/*
  Any bits set to 0 in this register send their respective 
  interrupts to the INT1 pin, whereas bits set to 1 send their 
  respective interrupts to the INT2 pin. All selected interrupts 
  for a given pin are OR’ed.
*/
  /* Read the data format register to preserve bits */
  uint8_t int_map = readRegister(ADXL345_REG_INT_MAP);

  // Set the interruptType corresponding Bit to the corresponding value for specific int Pin to enable
  if(intPin){
    int_map |= interruptType; // Set the corresponding Bit to 1
  }else{
    int_map &= ~interruptType; // Set the corresponding Bit to 0
  }

  //Serial.println("setInterruptPin: " + String(int_map));

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_INT_MAP, int_map);
}

void ADXL345::setInterruptPinsPolarity_ActiveHigh(bool intPin_active_high) 
{
/*
  A value of 0 in the INT_INVERT bit sets the interrupts to active high, 
  and a value of 1 sets the interrupts to active low.
*/
  /* Read the data format register to preserve bits */
  uint8_t data_format = readRegister(ADXL345_REG_DATA_FORMAT);

  // Set the D5 INT_INVERT Bit to the corresponding value
  if(intPin_active_high){
    data_format &= ~0b00100000; // Set the D5 INT_INVERT Bit to 0
  }else{
    data_format |=  0b00100000; // Set the D5 INT_INVERT Bit to 1
  }

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, data_format);
}

bool ADXL345::getInterruptStatus(interruptType_t interruptType)
{
  uint8_t int_source = readRegister(ADXL345_REG_INT_SOURCE);

  int_source &= interruptType; // Mask out only the Bit of specific Interrpt
  
  return (int_source != 0); // Returns true when masked out Bit is 1
}
