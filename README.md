# adxl345_tiny
(hopefully fast and tiny) Library for the I2C ADXL345 Accelerometer featuring methods for Interrupts and Low Power / Sleep mode configuration

The Library is based on the Adafruit's Library: https://github.com/adafruit/Adafruit_ADXL345/
which unfortiunately did not contain any methods for Interrupt or Low Power/ Sleep Mode configuration. I extended the Library with these functions. 
Furthermore I removed the dependency of the Unified Sensor Library objects (you directly get returned the raw 16-bit Integer values by the methods for each exis) and added a method `getRawAccelDataBurst()` for getting data of all three axis as fast as possible.

NOTE: This Library is in very early development stage so be a little forgiving. But I still wanted to share it at the current stage already, since I couldn't find any other ADXL345 Library out there which fitted my needs.

## Compatibility
Currently tested with:
- Arduino MKR NB 1500
- ESP32 (WROOM-32)

## Sensor Datasheet:
EN: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

##Example
