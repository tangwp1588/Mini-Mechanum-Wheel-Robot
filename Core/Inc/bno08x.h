#ifndef __BNO08X_H_
#define __BNO08X_H_
 
#include "main.h"
 
// The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B
#define BNO080_W_ADDRESS     ( BNO080_DEFAULT_ADDRESS << 1 | 0x00 )
#define BNO080_R_ADDRESS     ( BNO080_DEFAULT_ADDRESS << 1 | 0x01 )
 
// The catch-all default is 32
#define I2C_BUFFER_LENGTH 32
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 
// Registers
#define CHANNEL_COMMAND  0
#define CHANNEL_EXECUTABLE  1
#define CHANNEL_CONTROL  2
#define CHANNEL_REPORTS  3
#define CHANNEL_WAKE_REPORTS  4
#define CHANNEL_GYRO  5
 
// All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
// These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
 
// All the different sensors and features we can get reports from
// These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13

#define SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR 0x28

#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
 
// Record IDs from figure 29, page 29 reference manual
// These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B
 
// Command IDs from section 6.4, page 42
// These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11
 
#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5
 
#define MAX_PACKET_SIZE 128  // Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9  // This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)
void softReset();            // Try to reset the IMU via software
unsigned char resetReason(); // Query the IMU for the reason it last reset
 
float qToFloat(int16_t fixedPointValue, unsigned char qPoint); // Given a Q value, converts fixed point floating to regular floating point number
 
// char waitForI2C(); // Delay based polling for I2C traffic
char receivePacket(void);
char getData(unsigned int charsRemaining); // Given a number of chars, send the requests in I2C_BUFFER_LENGTH chunks
char sendPacket(unsigned char channelNumber, unsigned char dataLength);
// void printPacket(void); // Prints the current shtp header and data packets

void enableARVRStabilizedRotationVector(unsigned int timeBetweenReports);

void enableRotationVector(unsigned int timeBetweenReports);
void enableGameRotationVector(unsigned int timeBetweenReports);
void enableAccelerometer(unsigned int timeBetweenReports);
void enableLinearAccelerometer(unsigned int timeBetweenReports);
void enableGyro(unsigned int timeBetweenReports);
void enableMagnetometer(unsigned int timeBetweenReports);
void enableStepCounter(unsigned int timeBetweenReports);
void enableStabilityClassifier(unsigned int timeBetweenReports);
 
char dataAvailable(void);
void parseInputReport(void);
 
float getQuatI();
float getQuatJ();
float getQuatK();
float getQuatReal();
float getQuatRadianAccuracy();
unsigned char getQuatAccuracy();
 
float getAccelX();
float getAccelY();
float getAccelZ();
unsigned char getAccelAccuracy();
 
float getLinAccelX();
float getLinAccelY();
float getLinAccelZ();
unsigned char getLinAccelAccuracy();
 
float getGyroX();
float getGyroY();
float getGyroZ();
unsigned char getGyroAccuracy();
 
float getMagX();
float getMagY();
float getMagZ();
unsigned char getMagAccuracy();
 
void calibrateAccelerometer();
void calibrateGyro();
void calibrateMagnetometer();
void calibratePlanarAccelerometer();
void calibrateAll();
void endCalibration();
void saveCalibration();
 
unsigned int getStepCount();
unsigned char getStabilityClassifier();
unsigned char getActivityClassifier();
 
void setFeatureCommand(unsigned char reportID, unsigned int timeBetweenReports, unsigned long specificConfig);
void sendCommand(unsigned char command);
void sendCalibrateCommand(unsigned char thingToCalibrate);
 
// Metadata functions
int getQ1(unsigned int recordID);
int getQ2(unsigned int recordID);
int getQ3(unsigned int recordID);
float getResolution(unsigned int recordID);
float getRange(unsigned int recordID);
unsigned long readFRSword(unsigned int recordID, unsigned char wordNumber);
void frsReadRequest(unsigned int recordID, unsigned int readOffset, unsigned int blockSize);
char readFRSdata(unsigned int recordID, unsigned char startLocation, unsigned char wordsToRead);
 
void I2C_Init(void);
 
#endif