/* ITG3701_LSM303D_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: November 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The ITG3701 is an Invensense gyroscope that has a range of full scale settings including +/-4000 dps, making
 it the highest full-scale range gyroscope of those inexpensive MEMS gyros available and suitable for measuring
 all kinds of sports action like tennis swings, baseball bat swings, etc. as well as any rotations that
 approach a 10 Hz rotation rate, which is pretty fast for human or animal action. The gyro is coupled with ST Microelectronics
 LSM303D e-compass (the same accelerometer and magnetometer in the LSM9DS0) and a Measurement Specialties MS5637 
 pressure/temperature sensor for altimetry. The MS5637 is a simple but high resolution pressure sensor, which can be 
 used in its high resolution mode but with power consumption od 20 microAmp, or in a lower resolution mode with 
 power consumption of only 1 microAmp. The choice will depend on the application.
 This 10 DoF motion sensong solution is ideal for tracking vigorous human or animal motion.
 
 This sketch demonstrates basic ITG3701+LSM303D+MS5637 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the ITG3701+LSM303D+MS5637 Add-On Shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the ITG3701+LSM303D+MS5637 Teensy 3.1 add-on shield/breakout board.
 
 Hardware setup:
 ITG3701+LSM303DBreakout --Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 16
 SCL ----------------------- 17
 GND ---------------------- GND
 
 Note: The ITG3701+LSM303D+MS5637 are I2C sensors and can use the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using either a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
#define MS5637_RESET      0x1E
#define MS5637_CONVERT_D1 0x40
#define MS5637_CONVERT_D2 0x50
#define MS5637_ADC_READ   0x00

// See also ITG3701 Register Map and Descriptions
// http://www.invensense.com/mems/gyro/documents/RM-000001-ITG-3701-RM.pdf
//
////////////////////////////
// ITG3701 Gyro Registers //
////////////////////////////
#define ITG3701_XG_OFFS_TC_H     0x04
#define ITG3701_XG_OFFS_TC_L     0x05
#define ITG3701_YG_OFFS_TC_H     0x07
#define ITG3701_YG_OFFS_TC_L     0x08
#define ITG3701_ZG_OFFS_TC_H     0x0A
#define ITG3701_ZG_OFFS_TC_L     0x0B
#define ITG3701_XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope
#define ITG3701_XG_OFFS_USRL     0x14
#define ITG3701_YG_OFFS_USRH     0x15
#define ITG3701_YG_OFFS_USRL     0x16
#define ITG3701_ZG_OFFS_USRH     0x17
#define ITG3701_ZG_OFFS_USRL     0x18
#define ITG3701_SMPLRT_DIV       0x19
#define ITG3701_CONFIG           0x1A
#define ITG3701_GYRO_CONFIG      0x1B
#define ITG3701_FIFO_EN          0x23
#define ITG3701_INT_PIN_CFG      0x37
#define ITG3701_INT_ENABLE       0x38
#define ITG3701_INT_STATUS       0x3A
#define ITG3701_TEMP_OUT_H       0x41
#define ITG3701_TEMP_OUT_L       0x42
#define ITG3701_GYRO_XOUT_H      0x43
#define ITG3701_GYRO_XOUT_L      0x44
#define ITG3701_GYRO_YOUT_H      0x45
#define ITG3701_GYRO_YOUT_L      0x46
#define ITG3701_GYRO_ZOUT_H      0x47
#define ITG3701_GYRO_ZOUT_L      0x48
#define ITG3701_USER_CTRL        0x6A  
#define ITG3701_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define ITG3701_PWR_MGMT_2       0x6C
#define ITG3701_FIFO_COUNTH      0x72
#define ITG3701_FIFO_COUNTL      0x73
#define ITG3701_FIFO_R_W         0x74
#define ITG3701_WHO_AM_I         0x75 // Should return 0x68

// See also LSM303D Register Map and Descriptions
// http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00057547.pdf

//////////////////////////////////////////
//  LSM303D Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define  LSM303D_OUT_TEMP_L_XM	        0x05
#define  LSM303D_OUT_TEMP_H_XM	        0x06
#define  LSM303D_STATUS_REG_M		0x07
#define  LSM303D_OUT_X_L_M		0x08
#define  LSM303D_OUT_X_H_M		0x09
#define  LSM303D_OUT_Y_L_M		0x0A
#define  LSM303D_OUT_Y_H_M		0x0B
#define  LSM303D_OUT_Z_L_M		0x0C
#define  LSM303D_OUT_Z_H_M		0x0D
#define  LSM303D_WHO_AM_I_XM		0x0F
#define  LSM303D_INT_CTRL_REG_M	        0x12
#define  LSM303D_INT_SRC_REG_M	        0x13
#define  LSM303D_INT_THS_L_M		0x14
#define  LSM303D_INT_THS_H_M		0x15
#define  LSM303D_OFFSET_X_L_M		0x16
#define  LSM303D_OFFSET_X_H_M		0x17
#define  LSM303D_OFFSET_Y_L_M		0x18
#define  LSM303D_OFFSET_Y_H_M		0x19
#define  LSM303D_OFFSET_Z_L_M		0x1A
#define  LSM303D_OFFSET_Z_H_M		0x1B
#define  LSM303D_REFERENCE_X		0x1C
#define  LSM303D_REFERENCE_Y		0x1D
#define  LSM303D_REFERENCE_Z		0x1E
#define  LSM303D_CTRL_REG0_XM		0x1F
#define  LSM303D_CTRL_REG1_XM		0x20
#define  LSM303D_CTRL_REG2_XM		0x21
#define  LSM303D_CTRL_REG3_XM		0x22
#define  LSM303D_CTRL_REG4_XM		0x23
#define  LSM303D_CTRL_REG5_XM		0x24
#define  LSM303D_CTRL_REG6_XM		0x25
#define  LSM303D_CTRL_REG7_XM		0x26
#define  LSM303D_STATUS_REG_A		0x27
#define  LSM303D_OUT_X_L_A		0x28
#define  LSM303D_OUT_X_H_A		0x29
#define  LSM303D_OUT_Y_L_A		0x2A
#define  LSM303D_OUT_Y_H_A		0x2B
#define  LSM303D_OUT_Z_L_A		0x2C
#define  LSM303D_OUT_Z_H_A		0x2D
#define  LSM303D_FIFO_CTRL_REG	        0x2E
#define  LSM303D_FIFO_SRC_REG		0x2F
#define  LSM303D_INT_GEN_1_REG	        0x30
#define  LSM303D_INT_GEN_1_SRC	        0x31
#define  LSM303D_INT_GEN_1_THS	        0x32
#define  LSM303D_INT_GEN_1_DURATION	0x33
#define  LSM303D_INT_GEN_2_REG	        0x34
#define  LSM303D_INT_GEN_2_SRC	        0x35
#define  LSM303D_INT_GEN_2_THS	        0x36
#define  LSM303D_INT_GEN_2_DURATION	0x37
#define  LSM303D_CLICK_CFG		0x38
#define  LSM303D_CLICK_SRC		0x39
#define  LSM303D_CLICK_THS		0x3A
#define  LSM303D_TIME_LIMIT		0x3B
#define  LSM303D_TIME_LATENCY		0x3C
#define  LSM303D_TIME_WINDOW		0x3D
#define  LSM303D_ACT_THS		0x3E
#define  LSM303D_ACT_DUR		0x3F

// Using the ITG3701+LSM303D+MS5637 Teensy 3.1 Add-On shield, ADO is set to 0 
#define ADO 0
#if ADO
#define LSM303D_ADDRESS   0x1D // Address of accel/magnetometer when ADO = 1
#define ITG3701_ADDRESS   0x69 // Address of gyro when ADO = 1
#define MS5637_ADDRESS    0x76 // Address of altimeter
#else 
#define LSM303D_ADDRESS   0x1E // Address of accel/magnetometer when ADO = 0
#define ITG3701_ADDRESS   0x68 // Address of gyro when ADO = 0
#define MS5637_ADDRESS    0x76 // Address of altimeter
#endif

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_4G,
  AFS_6G,
  AFS_8G,
  AFS_16G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_3_125Hz,
  AODR_6_25Hz,
  AODR_12_5Hz,
  AODR_25Hz,
  AODR_50Hz,
  AODR_100Hz,
  AODR_200Hz,
  AODR_400Hz,
  AODR_800Hz,
  AODR_1600Hz
};

enum Abw {  // set of allowable accewl bandwidths
   ABW_773Hz = 0,
   ABW_194Hz,
   ABW_362Hz,
   ABW_50Hz
};

enum Gscale {
  GFS_500DPS = 0,
  GFS_1000DPS,
  GFS_2000DPS,
  GFS_4000DPS
};


enum Godr {  // set of allowable gyro sample rates
  GODR_95Hz = 0,
  GODR_190Hz,
  GODR_380Hz,
  GODR_760Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 12.5 Hz at Godr = 95 Hz, 12.5 Hz at Godr = 190 Hz,  30 Hz at Godr = 760 Hz
  GBW_med,      // 25 Hz   at Godr = 95 Hz, 25 Hz   at Godr = 190 Hz,  35 Hz at Godr = 760 Hz
  GBW_high,     // 25 Hz   at Godr = 95 Hz, 50 Hz   at Godr = 190 Hz,  50 Hz at Godr = 760 Hz
  GBW_highest   // 25 Hz   at Godr = 95 Hz, 70 Hz   at Godr = 190 Hz, 100 Hz at Godr = 760 Hz
};

enum Mscale {  // set of allowable mag full scale settings
  MFS_2G = 0,
  MFS_4G,
  MFS_8G,
  MFS_12G
};

enum Mres {
  MRES_LowResolution = 0, 
  MRES_NoOp,
  MRES_HighResolution
};

enum Modr {  // set of allowable mag sample rates
  MODR_3_125Hz = 0,
  MODR_6_25Hz,
  MODR_12_5Hz,
  MODR_25Hz,
  MODR_50Hz,
  MODR_100Hz
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor full scale
uint8_t OSR = ADC_8192;      // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_500DPS; // gyro full scale
uint8_t Godr = GODR_190Hz;   // gyro data sample rate
uint8_t Gbw = GBW_low;       // gyro data bandwidth
uint8_t Ascale = AFS_2G;     // accel full scale
uint8_t Aodr = AODR_200Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_2G;     // mag full scale
uint8_t Modr = MODR_25Hz;    // mag data sample rate
uint8_t Mres = MRES_HighResolution;  // magnetometer operation mode
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 15;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the ITG3701yro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
float SelfTest[3] = {0., 0., 0.};

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void setup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(4000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
/*  
  display.begin(); // Initialize the display
  display.setContrast(40); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("LSM303D");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
*/

  // Read the WHO_AM_I registers, this is a good test of communication
  Serial.println("ITG3701+LSM303D 9-axis motion sensor...");
  byte c = readByte(ITG3701_ADDRESS, ITG3701_WHO_AM_I);  // Read WHO_AM_I register for ITG3701 gyro
  Serial.println("ITG3701 gyro"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x68, HEX);
  byte d = readByte(LSM303D_ADDRESS, LSM303D_WHO_AM_I_XM);  // Read WHO_AM_I register for LSM303D accel/magnetometer
  Serial.println("LSM303D accel/magnetometer"); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x49, HEX);
/*
  display.setCursor(20,0); display.print("LSM303D");
  display.setCursor(0,10); display.print("I AM"); display.print(c, HEX);  
  display.setCursor(0,20); display.print("I Should Be"); display.print(0xD4, HEX); 
  display.setCursor(0,30); display.print("I AM"); display.print(d, HEX);  
  display.setCursor(0,40); display.print("I Should Be"); display.print(0x49, HEX); 
  display.display();
  delay(1000); 
  */

  if (c == 0x68 && d == 0x49) // WHO_AM_I should always be 0xD4 for the gyro and 0x49 for the accel/mag
  {  
    Serial.println("ITG3701+LSM303D is online...");
 
   initITG3701(); 
   Serial.println("ITG3701 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

   initLSM303D(); 
   Serial.println("LSM303D initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    
//    ITG3701SelfTest(SelfTest); // Start by performing self test and reporting values
//    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
//    delay(1000);
 
   // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   getMres();
   Serial.print("accel sensitivity is "); Serial.print(1./(1000.*aRes)); Serial.println(" LSB/mg");
   Serial.print("gyro sensitivity is "); Serial.print(1./(gRes)); Serial.println(" LSB/dps");
   Serial.print("mag sensitivity is "); Serial.print(1./(1000.*mRes)); Serial.println(" LSB/mGauss");

  // Calibrate gyro and accelerometers, load biases in bias registers
  gyrocalITG3701(gyroBias);
  accelcalLSM303D(accelBias); 
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
 
  magcalLSM303D(magBias);
  Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]);
  
  /* display.clearDisplay();
     
  display.setCursor(0, 0); display.print("LSM303Dbias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(66, 24); display.print("o/s");   
 
  display.display();
  delay(1000); 
 */ 
 
  // Reset the MS5637 pressure sensor
  MS5637Reset();
  delay(100);
  Serial.println("MS5637 pressure sensor reset...");
  // Read PROM data from MS5637 pressure sensor
  MS5637PromRead(Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(Pcal[0]);
  unsigned char refCRC = Pcal[0]  >> 12;
  Serial.print("C1 = "); Serial.println(Pcal[1]);
  Serial.print("C2 = "); Serial.println(Pcal[2]);
  Serial.print("C3 = "); Serial.println(Pcal[3]);
  Serial.print("C4 = "); Serial.println(Pcal[4]);
  Serial.print("C5 = "); Serial.println(Pcal[5]);
  Serial.print("C6 = "); Serial.println(Pcal[6]);
  Serial.print("C7 = "); Serial.println(Pcal[7]);
  
  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);  
  
/*  display.clearDisplay();
  display.setCursor(20,0); display.print("MS5637");
  display.setCursor(0,10); display.print("CRC is "); display.setCursor(50,10); display.print(nCRC);
  display.setCursor(0,20); display.print("Should be "); display.setCursor(50,30); display.print(refCRC);
  display.display();
  delay(1000);  
 */      
  }
  else
  {
    Serial.print("Could not connect to ITG3701+LSM303D: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  if (readByte(LSM303D_ADDRESS, LSM303D_STATUS_REG_A) & 0x08) {  // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
  } 
   
  if (readByte(ITG3701_ADDRESS, ITG3701_INT_STATUS) & 0x01) {  // check if new gyro data is ready  
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  }
  
  if (readByte(LSM303D_ADDRESS, LSM303D_STATUS_REG_M) & 0x08) {  // check if new mag data is ready  
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes - magBias[1];  
    mz = (float)magCount[2]*mRes - magBias[2];   
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x and y axes of the accelerometer/magnetometer and gyro are aligned. The magnetometer  
  // z-axis (+ updown) is anti-aligned with the z-axis (+ up) of accelerometer and gyro
  // We have to make no allowance for this orientation match in feeding the output to the quaternion filter.
  // For the LSM303D, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis  
  // This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my, -mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, -mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)1000*mx ); 
    Serial.print(" my = "); Serial.print( (int)1000*my ); 
    Serial.print(" mz = "); Serial.print( (int)1000*mz ); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    }               
    tempCount = readTempData();  // Read the gyro adc values
    temperature = ((float) tempCount/8. + 25.0); // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
 
    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
    
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
//
// Second order corrections
    if(Temperature > 20) 
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
 // End of second order corrections
 //
     Temperature = Temperature - T2/100;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or Pa/100
  
    float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));
    if(SerialDebug) {
    Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
     
    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }
/*   
    display.clearDisplay();    
 
    display.setCursor(0, 0); display.print(" x   y   z ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
 
/*    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz"); 
    display.display();
*/


    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gauss (00), 4 Gauss (01), 8 Gauss (10) and 12 Gauss (11)
    case MFS_2G:
          mRes = 2.0/32768.0;
          break;
    case MFS_4G:
          mRes = 4.0/32768.0;
          break;
    case MFS_8G:
          mRes = 8.0/32768.0;
          break;
    case MFS_12G:
          mRes = 12.0/32768.0;
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 500 DPS (00), 1000 DPS (01), 2000 DPS (10) and 4000 DPS  (11). 
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
    case GFS_4000DPS:
          gRes = 4000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (000), 4 Gs (001), 6 gs (010), 8 Gs (011), and 16 gs (100). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_6G:
          aRes = 6.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_A, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LSM303D_ADDRESS, LSM303D_OUT_TEMP_L_XM, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}
 
 
void initITG3701()
{  
 // wake up device
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the ITG3701, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(ITG3701_ADDRESS, ITG3701_CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(ITG3701_ADDRESS, ITG3701_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0] 
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

 // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(ITG3701_ADDRESS, ITG3701_INT_PIN_CFG, 0x20);    
   writeByte(ITG3701_ADDRESS, ITG3701_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


void initLSM303D()
{  
  // configure the accelerometer-specify ODR (sample rate) selection with Aodr, enable block data update
   writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG1_XM, Aodr << 4 | 0x0F);
   // configure the accelerometer-specify bandwidth and full-scale selection with Abw, Ascale 
   writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG2_XM, Abw << 6 | Ascale << 3);
    // enable temperature sensor, set magnetometer ODR (sample rate) and resolution mode
   writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG5_XM, 0x80 | Mres << 5 | Modr << 2);
   // set magnetometer full scale
   writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG6_XM, Mscale << 5 & 0x60);
   writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG7_XM, 0x00); // select continuous conversion mode
 }


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelcalLSM303D(float * dest1)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;
  

   Serial.println("Calibrating accel...");
 
  // now get the accelerometer bias
  byte c = readByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM);
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM, c | 0x40);     // Enable gyro FIFO  
  delay(200);                                                       // Wait for change to take effect
  writeByte(LSM303D_ADDRESS, LSM303D_FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(LSM303D_ADDRESS, LSM303D_FIFO_SRC_REG) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_A, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
  
  dest1[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest1[1] = (float)accel_bias[1]*aRes;
  dest1[2] = (float)accel_bias[2]*aRes;
  
  c = readByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM);
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM, c & ~0x40);   //Disable accel FIFO  
  delay(200);
  writeByte(LSM303D_ADDRESS, LSM303D_FIFO_CTRL_REG, 0x00);  // Enable accel bypass mode
}

void magcalLSM303D(float * dest1) 
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   sample_count = 128;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;   
    dest1[2] = (float) mag_bias[2]*mRes;          

 /* //write biases to accelerometermagnetometer offset registers as counts);
  writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
 */
   Serial.println("Mag Calibration done!");
}

// Function which accumulates gyro data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into gyro bias registers.
void gyrocalITG3701(float * dest1)
{  
  uint8_t data[6]; // data array to hold gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0};
  
 // reset device
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x01);  
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(ITG3701_ADDRESS, ITG3701_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(ITG3701_ADDRESS, ITG3701_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(ITG3701_ADDRESS, ITG3701_USER_CTRL, 0x00);    // Disable FIFO 
  writeByte(ITG3701_ADDRESS, ITG3701_USER_CTRL, 0x04);    // Reset FIFO 
  delay(15);
  
// Configure ITG3701 gyro for bias calculation
  writeByte(ITG3701_ADDRESS, ITG3701_CONFIG, 0x01);      // Set low-pass filter to 184 Hz
  writeByte(ITG3701_ADDRESS, ITG3701_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, 0x00); // Set gyro full-scale to 500 degrees per second, maximum sensitivity
 
  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(ITG3701_ADDRESS, ITG3701_USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(ITG3701_ADDRESS, ITG3701_FIFO_EN, 0x70);     // Enable gyro sensors for FIFO  (max size 512 bytes in ITG3701)
  delay(100); // accumulate 80 samples in 80 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(ITG3701_ADDRESS, ITG3701_FIFO_EN, 0x00);        // Disable gyro sensors for FIFO
  readBytes(ITG3701_ADDRESS, ITG3701_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = (uint16_t)((uint16_t)data[0] << 8) | data[1];
//  packet_count = fifo_count/6; // How many sets of full gyro data for averaging
  packet_count = 128; // How many sets of full gyro data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
//    readBytes(ITG3701_ADDRESS, ITG3701_FIFO_R_W, 6, &data[0]); // read data for averaging
    readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &data[0]); // read data for averaging
    gyro_temp[0]  = (int16_t) (((int16_t)data[0] << 8) | data[1]) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;
    
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
    delay(5);
}
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
      
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]  >> 8) & 0xFF;  
  data[1] = (-gyro_bias[0])       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1])       & 0xFF;
  data[4] = (-gyro_bias[2]  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2])       & 0xFF;
  
// Push gyro biases to hardware registers
//  writeByte(ITG3701_ADDRESS, ITG3701_XG_OFFS_USRH, data[0]);
//  writeByte(ITG3701_ADDRESS, ITG3701_XG_OFFS_USRL, data[1]);
//  writeByte(ITG3701_ADDRESS, ITG3701_YG_OFFS_USRH, data[2]);
//  writeByte(ITG3701_ADDRESS, ITG3701_YG_OFFS_USRL, data[3]);
//  writeByte(ITG3701_ADDRESS, ITG3701_ZG_OFFS_USRH, data[4]);
//  writeByte(ITG3701_ADDRESS, ITG3701_ZG_OFFS_USRL, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]*gRes;  
  dest1[1] = (float) gyro_bias[1]*gRes;
  dest1[2] = (float) gyro_bias[2]*gRes;
}

/*
// Gyroscope self test; check calibration wrt factory settings
void ITG3701SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[3];
   int16_t gAvg[3], gSTAvg[3];
   float factoryTrim[3];
   uint8_t FS = 0;
   
  writeByte(ITG3701_ADDRESS, ITG3701_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(ITG3701_ADDRESS, ITG3701_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
    readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
   readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(ITG3701_ADDRESS, ITG3701_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[1] = readByte(ITG3701_ADDRESS, ITG3701_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[2] = readByte(ITG3701_ADDRESS, ITG3701_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i];   // Report percent differences
   }
   
}
*/

// I2C communication with the MS5637 is a little different from that with the LSM303Dand most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

        void MS5637Reset()
        {
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
	Wire.write(MS5637_RESET);                // Put reset command in Tx buffer
	Wire.endTransmission();                  // Send the Tx buffer
        }
        
        void MS5637PromRead(uint16_t * destination)
        {
        uint8_t data[2] = {0,0};
        for (uint8_t ii = 0; ii <8; ii++) {
          Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
          Wire.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
          Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	  uint8_t i = 0;
          Wire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address 
	  while (Wire.available()) {
          data[i++] = Wire.read(); }               // Put read results in the Rx buffer
          destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
        }
        }

        uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
        {
        uint8_t data[3] = {0,0,0};
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
        Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
        
        switch (OSR)
        {
          case ADC_256: delay(1); break;  // delay for conversion to complete
          case ADC_512: delay(3); break;
          case ADC_1024: delay(4); break;
          case ADC_2048: delay(6); break;
          case ADC_4096: delay(10); break;
          case ADC_8192: delay(20); break;
        }
       
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(0x00);                        // Put ADC read command in Tx buffer
        Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address 
	while (Wire.available()) {
        data[i++] = Wire.read(); }               // Put read results in the Rx buffer
        return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
        }



unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;
  
  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for(cnt = 0; cnt < 16; cnt++)
  {
    if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
    for(n_bit = 8; n_bit > 0; n_bit--)
    {
        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
        else                  n_rem = (n_rem<<1);
    }
  }
  n_rem = ((n_rem>>12) & 0x000F);
  return (n_rem ^ 0x00);
}



// I2C read/write functions for the LSM303Dand AK8963 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(0x80 | subAddress);     // Put slave register address in Tx buffer, include 0x80 for LSM303D multiple byte read
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

