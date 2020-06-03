# adxl345_tiny
(hopefully fast and tiny) Library for the I2C ADXL345 Accelerometer featuring methods for interrupts and Low Power / Sleep mode configuration which are currently missing in other libraries

The Library is based on the Adafruit's Library: https://github.com/adafruit/Adafruit_ADXL345/
which unfortunately did not contain any methods for Interrupt or Low Power/ Sleep Mode configuration. I extended the Library with these functions. 
Furthermore I removed the dependency of the Unified Sensor Library objects (you directly get returned the raw 16-bit Integer values by the methods for each axis) and added a method `getRawAccelDataBurst()` for getting data of all three axis as fast as possible. Since I removed the Unified Sensor Library "bloat" and implemendet some more fast methods and placed everything in a single library, I decided to call it tiny even if there is more functionality ;)

__NOTE__: This Library is in very early development stage so be a little forgiving. But I still wanted to share it at the current stage already, since I couldn't find any other ADXL345 Library out there which fitted my needs so it may be helpful for you.

## Compatibility
Currently tested with:
- Arduino MKR NB 1500
- ESP32 (WROOM-32)

## Sensor Datasheet:
EN: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

## Example
```c++
// Initialization
ADXL345 accel = ADXL345(12345); // (Sensor_ID)

if(!accel.begin())
{
  Serial.println("No ADXL345 detected!");
  while(1); 
}
  
// Basic configuration
accel.setRange(ADXL345_RANGE_16_G); 
accel.setDataRate(ADXL345_DATARATE_800_HZ);
accel.setSleepDataRate(ADXL345_SLEEPDATARATE_8_HZ); // Data refreshment / sample rate used during sleep. 8 Hz is max.

// Configuration of Activity Interrupt (wakes up the accel from sleep)
accel.setInterruptPinsPolarity_ActiveHigh(true); // Interrut Pins are HI when interrupt is fired (default)
accel.setActivityDetectionAxis(ADXL345_DETECT_MODE_AC, false, false, true); // (MODE, x_axis, y_axis, z_axis) Sets the axes which should be watched for exceeding specified limit for event triggering
accel.setActivityInterrupt(2, ADXL345_INT2_PIN, true); // (Threshold, Pin, Active) Enables/disables interrupt/event and sets the threshold and the Pin on which interrupt is fired

// Configuration of NO-Activity Interrupt (sets the accel to sleep)
accel.setInActivityDetectionAxis(ADXL345_DETECT_MODE_AC, false, false, true); // Configuration of which axes should be watched
accel.setInterrupt(ADXL345_INTERRUPT_Inactivity, true); // Even if you don't want to fire a interrupt when going to sleep, you have to configure (and firing one) on one of the interrupt pins. (Interrupt is fired on the default Pin1, you can configure the interrupt to fire on an different using setInterruptPin)
accel.setAutoSleep(3, 10, true); // (Threshold, No_Activity_Timespan_Seconds, Active)

//Configuration of Data_Ready Interrupt
accel.setInterruptPin(ADXL345_INTERRUPT_DATA_READY, ADXL345_INT1_PIN);
accel.setInterrupt(ADXL345_INTERRUPT_DATA_READY, true);


// MICROCONTROLLER CONFIGURATION
// Set Pin 1 of an Arduino MKR NB 1500 to wake up the the controller by Interrupt sent by ADXL345 
pinMode(1, INPUT);
LowPower.attachInterruptWakeup(1, wakeupInterrupt_Mthod, CHANGE);

// Setting the Pin 0 of an Arduino MKR NB 1500 for calling the controller's ISR whenever ADXL345 Data_Ready Interrupt respectively an interrupt on the configured Pin is received.
pinMode(0, INPUT);
attachInterrupt(digitalPinToInterrupt(0), interruptDataReady_ISR, RISING);
```
