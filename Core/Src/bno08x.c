#include "bno08x.h"
#include "math.h"
#include "i2c.h"
#include <stdint.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

// Global Variables
unsigned char shtpHeader[4]; // Each packet has a header of 4 chars
unsigned char shtpData[MAX_PACKET_SIZE];
unsigned char sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; // There are 6 com channels. Each channel has its own seqnum
unsigned char commandSequenceNumber = 0;              // Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
unsigned long metaData[MAX_METADATA_SIZE];                 // There is more than 10 words in a metadata record but we'll stop at Q point 3
// Variables
unsigned char _deviceAddress;//跟踪 I2C 地址。setI2CAddress 更改了这一点。


// These are the raw sensor values pulled from the user requested Input Report
unsigned int rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
unsigned int rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
unsigned int rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
unsigned int rawMagX, rawMagY, rawMagZ, magAccuracy;
unsigned int rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
unsigned int stepCount;
unsigned char stabilityClassifier;
unsigned char activityClassifier;
unsigned char *_activityConfidences; // Array that store the confidences of the 9 possible activities

// These Q values are defined in the datasheet but can also be obtained by querying the meta data records
// See the read metadata example for more info
int rotationVector_Q1 = 14;
int accelerometer_Q1 = 8;
int linear_accelerometer_Q1 = 8;
int gyro_Q1 = 9;
int magnetometer_Q1 = 4;

char dataAvailable(void)
{
    if (receivePacket() == 1)
    {
        // Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            parseInputReport(); // This will update the rawAccelX, etc variables depending on which feature report is found
            return (1);
        }
    }
    return (0);
}

// This function pulls the data from the input report
// The input reports vary in length so this function stores the various 16-bit values as globals

// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
// shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
// shtpData[5 + 1]: Sequence number (See 6.5.18.2)
// shtpData[5 + 2]: Status
// shtpData[3]: HAL_Delay
// shtpData[4:5]: i/accel x/gyro x/etc
// shtpData[6:7]: j/accel y/gyro y/etc
// shtpData[8:9]: k/accel z/gyro z/etc
// shtpData[10:11]: real/gyro temp/etc
// shtpData[12:13]: Accuracy estimate
void parseInputReport(void)
{
    // Calculate the number of data bytes in this packet
    int dataLength = ((unsigned int)shtpHeader[1] << 8 | shtpHeader[0]);

    unsigned char status = shtpData[5 + 2] & 0x03; // Get status bits
    unsigned int data1 = (unsigned int)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    unsigned int data2 = (unsigned int)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    unsigned int data3 = (unsigned int)shtpData[5 + 9] << 8 | shtpData[5 + 8];
    unsigned int data4 = 0;
    unsigned int data5 = 0;
    unsigned char x = 0;
    dataLength &= ~(1 << 15); // Clear the MSbit. This bit indicates if this package is a continuation of the last.
    // Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; // Remove the header bytes from the data count

    if (dataLength - 5 > 9)
    {
        data4 = (unsigned int)shtpData[5 + 11] << 8 | shtpData[5 + 10];
    }
    if (dataLength - 5 > 11)
    {
        data5 = (unsigned int)shtpData[5 + 13] << 8 | shtpData[5 + 12];
    }

    // Store these generic values to their proper global variable
    if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
    {
        accelAccuracy = status;
        rawAccelX = data1;
        rawAccelY = data2;
        rawAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
    {
        accelLinAccuracy = status;
        rawLinAccelX = data1;
        rawLinAccelY = data2;
        rawLinAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
    {
        gyroAccuracy = status;
        rawGyroX = data1;
        rawGyroY = data2;
        rawGyroZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
    {
        magAccuracy = status;
        rawMagX = data1;
        rawMagY = data2;
        rawMagZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
             shtpData[5] == SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR)
    {
        quatAccuracy = status;
        rawQuatI = data1;
        rawQuatJ = data2;
        rawQuatK = data3;
        rawQuatReal = data4;
        rawQuatRadianAccuracy = data5; // Only available on rotation vector, not game rot vector
    }
    else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
    {
        stepCount = data3; // Bytes 8/9
    }
    else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
    {
        stabilityClassifier = shtpData[5 + 4]; // Byte 4 only
    }
    else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
    {
        activityClassifier = shtpData[5 + 5]; // Most likely state

        // Load activity classification confidences into the array
        for (x = 0; x < 9; x++)                            // Hardcoded to max of 9. TODO - bring in array size
            _activityConfidences[x] = shtpData[5 + 6 + x]; // 5 bytes of timestamp, byte 6 is first confidence byte
    }
    else
    {
        // This sensor report ID is unhandled.
        // See reference manual to add additional feature reports as needed
    }

    // TODO additional feature reports may be strung together. Parse them all.
}

// Return the rotation vector quaternion I
float getQuatI()
{
    float quat = qToFloat(rawQuatI, (unsigned char)rotationVector_Q1);
    return (quat);
}

// Return the rotation vector quaternion J
float getQuatJ()
{
    float quat = qToFloat(rawQuatJ, (unsigned char)rotationVector_Q1);
    return (quat);
}

// Return the rotation vector quaternion K
float getQuatK()
{
    float quat = qToFloat(rawQuatK, (unsigned char)rotationVector_Q1);
    return (quat);
}

// Return the rotation vector quaternion Real
float getQuatReal()
{
    float quat = qToFloat(rawQuatReal, (unsigned char)rotationVector_Q1);
    return (quat);
}

// Return the rotation vector accuracy
float getQuatRadianAccuracy()
{
    float quat = qToFloat(rawQuatRadianAccuracy, (unsigned char)rotationVector_Q1);
    return (quat);
}

// Return the acceleration component
unsigned char getQuatAccuracy()
{
    return (quatAccuracy);
}

// Return the acceleration component
float getAccelX()
{
    float accel = qToFloat(rawAccelX, (unsigned char)accelerometer_Q1);
    return (accel);
}

// Return the acceleration component
float getAccelY()
{
    float accel = qToFloat(rawAccelY, (unsigned char)accelerometer_Q1);
    return (accel);
}

// Return the acceleration component
float getAccelZ()
{
    float accel = qToFloat(rawAccelZ, (unsigned char)accelerometer_Q1);
    return (accel);
}

// Return the acceleration component
unsigned char getAccelAccuracy()
{
    return (accelAccuracy);
}

// linear acceleration, i.e. minus gravity

// Return the acceleration component
float getLinAccelX()
{
    float accel = qToFloat(rawLinAccelX, (unsigned char)linear_accelerometer_Q1);
    return (accel);
}

// Return the acceleration component
float getLinAccelY()
{
    float accel = qToFloat(rawLinAccelY, (unsigned char)linear_accelerometer_Q1);
    return (accel);
}

// Return the acceleration component
float getLinAccelZ()
{
    float accel = qToFloat(rawLinAccelZ, (unsigned char)linear_accelerometer_Q1);
    return (accel);
}

// Return the acceleration component
unsigned char getLinAccelAccuracy()
{
    return (accelLinAccuracy);
}

// Return the gyro component
float getGyroX()
{
    float gyro = qToFloat(rawGyroX, (unsigned char)gyro_Q1);
    return (gyro);
}

// Return the gyro component
float getGyroY()
{
    float gyro = qToFloat(rawGyroY, (unsigned char)gyro_Q1);
    return (gyro);
}

// Return the gyro component
float getGyroZ()
{
    float gyro = qToFloat(rawGyroZ, (unsigned char)gyro_Q1);
    return (gyro);
}

// Return the gyro component
unsigned char getGyroAccuracy()
{
    return (gyroAccuracy);
}

// Return the magnetometer component
float getMagX()
{
    float mag = qToFloat(rawMagX, (unsigned char)magnetometer_Q1);
    return (mag);
}

// Return the magnetometer component
float getMagY()
{
    float mag = qToFloat(rawMagY, (unsigned char)magnetometer_Q1);
    return (mag);
}

// Return the magnetometer component
float getMagZ()
{
    float mag = qToFloat(rawMagZ, (unsigned char)magnetometer_Q1);
    return (mag);
}

// Return the mag component
unsigned char getMagAccuracy()
{
    return (magAccuracy);
}

// Return the step count
unsigned int getStepCount()
{
    return (stepCount);
}

// Return the stability classifier
unsigned char getStabilityClassifier()
{
    return (stabilityClassifier);
}

// Return the activity classifier
unsigned char getActivityClassifier()
{
    return (activityClassifier);
}

// Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
// Q1 is used for all sensor data calculations
int getQ1(unsigned int recordID)
{
    // Q1 is always the lower 16 bits of word 7
    unsigned int q = readFRSword(recordID, 7) & 0xFFFF; // Get word 7, lower 16 bits
    return (q);
}

// Given a record ID, read the Q2 value from the metaData record in the FRS
// Q2 is used in sensor bias
int getQ2(unsigned int recordID)
{
    // Q2 is always the upper 16 bits of word 7
    unsigned int q = readFRSword(recordID, 7) >> 16; // Get word 7, upper 16 bits
    return (q);
}

// Given a record ID, read the Q3 value from the metaData record in the FRS
// Q3 is used in sensor change sensitivity
int getQ3(unsigned int recordID)
{
    // Q3 is always the upper 16 bits of word 8
    unsigned int q = readFRSword(recordID, 8) >> 16; // Get word 8, upper 16 bits
    return (q);
}

// Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float getResolution(unsigned int recordID)
{
    // The resolution Q value are 'the same as those used in the sensor's input report'
    // This should be Q1.
    int Q = getQ1(recordID);

    // Resolution is always word 2
    unsigned long value = readFRSword(recordID, 2); // Get word 2

    float resolution = qToFloat((unsigned int)value, (unsigned char)Q);

    return (resolution);
}

// Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float getRange(unsigned int recordID)
{
    // The resolution Q value are 'the same as those used in the sensor's input report'
    // This should be Q1.
    int Q = getQ1(recordID);

    // Range is always word 1
    unsigned long value = readFRSword(recordID, 1); // Get word 1

    float range = qToFloat((unsigned int)value, (unsigned char)Q);

    return (range);
}

// Given a record ID and a word number, look up the word data
// Helpful for pulling out a Q value, range, etc.
// Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
unsigned long readFRSword(unsigned int recordID, unsigned char wordNumber)
{
    if (readFRSdata(recordID, wordNumber, 1) == 1) // Get word number, just one word in length from FRS
        return (metaData[0]);                      // Return this one word

    return (0); // Error
}

// Ask the sensor for data from the Flash Record System
// See 6.3.6 page 40, FRS Read Request
void frsReadRequest(unsigned int recordID, unsigned int readOffset, unsigned int blockSize)
{
    shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; // FRS Read Request
    shtpData[1] = 0;                            // Reserved
    shtpData[2] = (readOffset >> 0) & 0xFF;     // Read Offset LSB
    shtpData[3] = (readOffset >> 8) & 0xFF;     // Read Offset MSB
    shtpData[4] = (recordID >> 0) & 0xFF;       // FRS Type LSB
    shtpData[5] = (recordID >> 8) & 0xFF;       // FRS Type MSB
    shtpData[6] = (blockSize >> 0) & 0xFF;      // Block size LSB
    shtpData[7] = (blockSize >> 8) & 0xFF;      // Block size MSB

    // Transmit packet on channel 2, 8 bytes
    sendPacket(CHANNEL_CONTROL, 8);
}

// Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
// Returns 1 if metaData array is loaded successfully
// Returns 0 if failure
char readFRSdata(unsigned int recordID, unsigned char startLocation, unsigned char wordsToRead)
{
    unsigned char spot = 0;
    unsigned char dataLength = 0;
    unsigned char frsStatus = 0;
    unsigned long data0 = 0;
    unsigned long data1 = 0;
    // First we send a Flash Record System (FRS) request
    frsReadRequest(recordID, startLocation, wordsToRead); // From startLocation of record, read a # of words

    // Read bytes until FRS reports that the read is complete
    while (1)
    {
        // Now we wait for response
        while (1)
        {
            unsigned char counter = 0;
            while (receivePacket() == 0)
            {
                if (counter++ > 100)
                    return (0); // Give up
                HAL_Delay(1);
            }

            // We have the packet, inspect it for the right contents
            // See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
            if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
                if (((unsigned int)shtpData[13] << 8 | shtpData[12]) == recordID)
                    break; // This packet is one we are looking for
        }

        dataLength = shtpData[1] >> 4;
        frsStatus = shtpData[1] & 0x0F;

        data0 = (unsigned long)shtpData[7] << 24 | (unsigned long)shtpData[6] << 16 | (unsigned long)shtpData[5] << 8 | (unsigned long)shtpData[4];
        data1 = (unsigned long)shtpData[11] << 24 | (unsigned long)shtpData[10] << 16 | (unsigned long)shtpData[9] << 8 | (unsigned long)shtpData[8];

        // Record these words to the metaData array
        if (dataLength > 0)
        {
            metaData[spot++] = data0;
        }
        if (dataLength > 1)
        {
            metaData[spot++] = data1;
        }

        if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
        {
            return (1); // FRS status is read completed! We're done!
        }
    }
}

// Send command to reset IC
// Read all advertisement packets from sensor
// The sensor has been seen to reset twice if we attempt too much too quickly.
// This seems to work reliably.
void softReset(void)
{
    shtpData[0] = 1; // Reset

    // Attempt to start communication with sensor
    sendPacket(CHANNEL_EXECUTABLE, 1); // Transmit packet on channel 1, 1 byte

    // Read all incoming data and flush it
    HAL_Delay(50);
    while (receivePacket() == 1)
        ;
    HAL_Delay(50);
    while (receivePacket() == 1)
        ;
}

// Get the reason for the last reset
// 1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
unsigned char resetReason()
{
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; // Request the product ID and reset info
    shtpData[1] = 0;                              // Reserved

    // Transmit packet on channel 2, 2 bytes
    sendPacket(CHANNEL_CONTROL, 2);

    // Now we wait for response
    if (receivePacket() == 1)
    {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            return (shtpData[1]);
        }
    }

    return (0);
}

// Given a register value and a Q point, convert to float
float qToFloat(int16_t fixedPointValue, unsigned char qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return (qFloat);
}

void enableARVRStabilizedRotationVector(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ARVR_STABILIZED_ROTATION_VECTOR, timeBetweenReports, 0);
}

// Sends the packet to enable the rotation vector
void enableRotationVector(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

// Sends the packet to enable the rotation vector
void enableGameRotationVector(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

// Sends the packet to enable the accelerometer
void enableAccelerometer(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports, 0);
}

// Sends the packet to enable the accelerometer
void enableLinearAccelerometer(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

// Sends the packet to enable the gyro
void enableGyro(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports, 0);
}

// Sends the packet to enable the magnetometer
void enableMagnetometer(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

// Sends the packet to enable the step counter
void enableStepCounter(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports, 0);
}

// Sends the packet to enable the Stability Classifier
void enableStabilityClassifier(unsigned int timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports, 0);
}

// Sends the commands to begin calibration of the accelerometer
void calibrateAccelerometer()
{
    sendCalibrateCommand(CALIBRATE_ACCEL);
}

// Sends the commands to begin calibration of the gyro
void calibrateGyro()
{
    sendCalibrateCommand(CALIBRATE_GYRO);
}

// Sends the commands to begin calibration of the magnetometer
void calibrateMagnetometer()
{
    sendCalibrateCommand(CALIBRATE_MAG);
}

// Sends the commands to begin calibration of the planar accelerometer
void calibratePlanarAccelerometer()
{
    sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

// See 2.2 of the Calibration Procedure document 1000-4044
void calibrateAll()
{
    sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void endCalibration()
{
    sendCalibrateCommand(CALIBRATE_STOP); // Disables all calibrations
}

// Given a sensor's report ID, this tells the BNO080 to begin reporting the values
// Also sets the specific config word. Useful for personal activity classifier
void setFeatureCommand(unsigned char reportID, unsigned int timeBetweenReports, unsigned long specificConfig)
{
    long microsBetweenReports = (long)timeBetweenReports * 1000L;

    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;     // Set feature command. Reference page 55
    shtpData[1] = reportID;                            // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0;                                   // Feature flags
    shtpData[3] = 0;                                   // Change sensitivity (LSB)
    shtpData[4] = 0;                                   // Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  // Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; // Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; // Report interval (MSB)
    shtpData[9] = 0;                                   // Batch Interval (LSB)
    shtpData[10] = 0;                                  // Batch Interval
    shtpData[11] = 0;                                  // Batch Interval
    shtpData[12] = 0;                                  // Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF;       // Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF;       // Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF;      // Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF;      // Sensor-specific config (MSB)

    // Transmit packet on channel 2, 17 bytes
    sendPacket(CHANNEL_CONTROL, 17);
}

// Tell the sensor to do a command
// See 6.3.8 page 41, Command request
// The caller is expected to set P0 through P8 prior to calling
void sendCommand(unsigned char command)
{
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; // Command Request
    shtpData[1] = commandSequenceNumber++;     // Increments automatically each function call
    shtpData[2] = command;                     // Command

    // Caller must set these
    /*shtpData[3] = 0; //P0
    shtpData[4] = 0; //P1
    shtpData[5] = 0; //P2
    shtpData[6] = 0;
    shtpData[7] = 0;
    shtpData[8] = 0;
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;*/

    // Transmit packet on channel 2, 12 bytes
    sendPacket(CHANNEL_CONTROL, 12);
}

// This tells the BNO080 to begin calibrating
// See page 50 of reference manual and the 1000-4044 calibration doc
void sendCalibrateCommand(unsigned char thingToCalibrate)
{
    unsigned char x = 0;
    /*shtpData[3] = 0; //P0 - Accel Cal Enable
    shtpData[4] = 0; //P1 - Gyro Cal Enable
    shtpData[5] = 0; //P2 - Mag Cal Enable
    shtpData[6] = 0; //P3 - Subcommand 0x00
    shtpData[7] = 0; //P4 - Planar Accel Cal Enable
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (x = 3; x < 12; x++) // Clear this section of the shtpData array
        shtpData[x] = 0;

    if (thingToCalibrate == CALIBRATE_ACCEL)
        shtpData[3] = 1;
    else if (thingToCalibrate == CALIBRATE_GYRO)
        shtpData[4] = 1;
    else if (thingToCalibrate == CALIBRATE_MAG)
        shtpData[5] = 1;
    else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
        shtpData[7] = 1;
    else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
    {
        shtpData[3] = 1;
        shtpData[4] = 1;
        shtpData[5] = 1;
    }
    else if (thingToCalibrate == CALIBRATE_STOP)
        ; // Do nothing, bytes are set to zero

    // Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

// This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
// See page 49 of reference manual and the 1000-4044 calibration doc
void saveCalibration()
{
    unsigned char x = 0;
    /*shtpData[3] = 0; //P0 - Reserved
    shtpData[4] = 0; //P1 - Reserved
    shtpData[5] = 0; //P2 - Reserved
    shtpData[6] = 0; //P3 - Reserved
    shtpData[7] = 0; //P4 - Reserved
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (x = 3; x < 12; x++) // Clear this section of the shtpData array
        shtpData[x] = 0;

    // Using this shtpData packet, send a command
    sendCommand(COMMAND_DCD); // Save DCD command
}





//  IIC 硬件底层

void I2C_Init(void)
{
    MX_I2C1_Init();
}

void WriteNbyte(u8 *p, u8 number) /*  WordAddress,First Data Address,Byte lenth   */
{
    HAL_I2C_Master_Transmit( &hi2c1, BNO080_W_ADDRESS, p, number, 0xffff );
}

void ReadNbyte(u8 *p, u8 number) /*  WordAddress,First Data Address,Byte lenth   */
{
    HAL_I2C_Master_Receive( &hi2c1, BNO080_R_ADDRESS, p, number, 0xffff );
}

u8 i2c_tmp[32] = {0};

// 检查是否有任何可用的新数据
// 将传入数据包的内容读入 shtpData 数组
char receivePacket(void)
{
    int dataLength = 0;
    unsigned char packetLSB = 0;
    unsigned char packetMSB = 0;
    unsigned char channelNumber = 0;
    unsigned char sequenceNumber = 0;
    ReadNbyte(i2c_tmp, 4);

    // Get the first four bytes, aka the packet header
    packetLSB = i2c_tmp[0];
    packetMSB = i2c_tmp[1];
    channelNumber = i2c_tmp[2];
    sequenceNumber = i2c_tmp[3]; // Not sure if we need to store this or not

    // Store the header info.
    shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber;

    // Calculate the number of data bytes in this packet
    dataLength = ((unsigned int)packetMSB << 8 | packetLSB);
    dataLength &= ~(1 << 15);//清除 MSbit。
//此位指示此包是否是上一个包的延续。暂时忽略它。
//TODO 将此捕获为错误并退出
    if (dataLength == 0)
    {
        // Packet is empty
        return (0); // All done
    }
    dataLength -= 4; // Remove the header bytes from the data count

    getData(dataLength);

    return (1); // We're done!
}
 
// 向传感器发送多个请求，直到从传感器接收到所有数据字节
// shtpData 缓冲区的最大容量为 MAX_PACKET_SIZE。超过此量的任何字节都将丢失。
// Arduino I2C 读取限制为 32 字节。标头是 4 个字节，因此我们每个交互可以读取的最大数据为 28 字节
char getData(unsigned int bytesRemaining)
{
    unsigned int dataSpot = 0; // Start at the beginning of shtpData array
    unsigned char x = 0;
    unsigned char incoming = 0;
    // Setup a series of chunked 32 byte reads
    while (bytesRemaining > 0)
    {
        unsigned int numberOfBytesToRead = bytesRemaining;
        if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
            numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);
        ReadNbyte(i2c_tmp, (unsigned char)(numberOfBytesToRead + 4));
 
        // 前四个字节是标头字节，被丢弃
 
        for (x = 0; x < numberOfBytesToRead; x++)
        {
            incoming = i2c_tmp[x + 4];
            if (dataSpot < MAX_PACKET_SIZE)
            {
                shtpData[dataSpot++] = incoming; // Store data into the shtpData array
            }
            else
            {
                // Do nothing with the data
            }
        }
 
        bytesRemaining -= numberOfBytesToRead;
    }
    return (1); // Done!
}
 
// 给定数据包，发送标头，然后发送数据
// 如果传感器不 ACK，则返回 0
// TODO - Arduino 的最大发送量为 32 字节。如果需要，将发送分解为多个数据包。
char sendPacket(unsigned char channelNumber, unsigned char dataLength)
{
    unsigned char packetLength = dataLength + 4; // Add four bytes for the header
    // if(packetLength > I2C_BUFFER_LENGTH) return(0); //You are trying to send too much. Break into smaller packets.
    unsigned char i = 0;
    // Send the 4 byte packet header
    i2c_tmp[0] = (packetLength & 0xFF);             // Packet length LSB
    i2c_tmp[1] = (0);                               // Packet length MSB
    i2c_tmp[2] = (channelNumber);                   // Channel number
    i2c_tmp[3] = (sequenceNumber[channelNumber]++); // Send the sequence number, increments with each packet sent, different counter for each channel
 
    // Send the user's data packet
    for (i = 0; i < dataLength; i++)
    {
        i2c_tmp[4 + i] = (shtpData[i]);
    }
    WriteNbyte(i2c_tmp, (u8)(dataLength + 4));
 
    return (1);
}