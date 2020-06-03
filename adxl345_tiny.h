/**************************************************************************/
/*!
    @file     adxl345_tiny.h
    @author   Steffen Schmelter
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

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADXL345_DEFAULT_ADDRESS     (0x53)  // Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60µA IDD
  ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50µA IDD
  ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45µA IDD
  ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40µA IDD
  ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34µA IDD
  ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23µA IDD (default value)
} dataRate_t;

/* Used with register 0x2D (ADXL345_REG_POWER_CTL) to set bandwidth */
typedef enum
{
  ADXL345_SLEEPDATARATE_8_HZ   = 0b00,
  ADXL345_SLEEPDATARATE_4_HZ   = 0b01,
  ADXL345_SLEEPDATARATE_2_HZ   = 0b10,
  ADXL345_SLEEPDATARATE_1_HZ   = 0b11
} sleepDataRate_t;

typedef enum
{
  ADXL345_INTERRUPT_DATA_READY   = 0b10000000,
  ADXL345_INTERRUPT_SINGLE_TAP   = 0b01000000,
  ADXL345_INTERRUPT_DOUBLE_TAP   = 0b00100000,
  ADXL345_INTERRUPT_Activity     = 0b00010000,
  ADXL345_INTERRUPT_Inactivity   = 0b00001000,
  ADXL345_INTERRUPT_FREE_FALL    = 0b00000100,
  ADXL345_INTERRUPT_Watermark    = 0b00000010,
  ADXL345_INTERRUPT_Overrun      = 0b00000001
} interruptType_t;

typedef enum
{
  ADXL345_INT1_PIN    = 0b0,
  ADXL345_INT2_PIN    = 0b1
} interruptPin_t;

typedef enum
{
  ADXL345_DETECT_MODE_AC   = 0b1,
  ADXL345_DETECT_MODE_DC   = 0b0
} detectMode_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

typedef struct {
  int16_t acceleration_x;
  int16_t acceleration_y;
  int16_t acceleration_z;
} accel_data_t;


class ADXL345 {
  public:
    ADXL345(int32_t sensorID = -1);

    bool       begin(uint8_t addr = ADXL345_DEFAULT_ADDRESS, bool resetToDefault = true);
    void       resetToDefaultValues(void);
    void       setRange(range_t range);
    range_t    getRange(void);
    void       setDataRate(dataRate_t dataRate);
    dataRate_t getDataRate(void);
    bool       getRawAccelData(accel_data_t*);
    bool       getRawAccelDataBurst(accel_data_t*);

    void       setAutoSleep(uint8_t acc_threshold, uint8_t time, bool autosleepEnabled);
    void       setSleepDataRate(sleepDataRate_t sleeptDataRate);
    bool       getSleepState(void);

    void       setActivityInterrupt(uint8_t acc_threshold, interruptPin_t intPin, bool interruptEnabled);
    void       setActivityDetectionAxis(detectMode_t detection_mode, bool ACT_X_enable, bool ACT_Y_enable, bool ACT_Z_enable);
    void       setInActivityDetectionAxis(detectMode_t detection_mode, bool INACT_X_enable, bool INACT_Y_enable, bool INACT_Z_enable);

    void       setInterrupt(interruptType_t interruptType, bool interruptEnabled);
    void       setInterruptPin(interruptType_t interruptType, interruptPin_t intPin);
    void       setInterruptPinsPolarity_ActiveHigh(bool intPin_active_high);
    bool       getInterruptStatus(interruptType_t interruptType);

    uint8_t    getDeviceID(void);
    void       writeRegister(uint8_t reg, uint8_t value);
    uint8_t    readRegister(uint8_t reg);
    bool       readRegisterBit(uint8_t reg, uint8_t bitToRead);
    int16_t    read16(uint8_t reg);
    int16_t    getRawX(void), getRawY(void), getRawZ(void);

  private:
    inline uint8_t  i2cread(void);
    inline void     i2cwrite(uint8_t x);

    int32_t _sensorID;
    range_t _range;
    int8_t  _i2caddr;
};