#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <L3G.h>


/* Taken from ADAFRUIT Library */
#define HMC5883_ADDRESS_MAG (0x3C >> 1) // 0011110x // I2C ADDRESS/BITS

/* REGISTERS */
typedef enum
{
    HMC5883_REGISTER_MAG_CRA_REG_M = 0x00,
    HMC5883_REGISTER_MAG_CRB_REG_M = 0x01,
    HMC5883_REGISTER_MAG_MR_REG_M = 0x02,
    HMC5883_REGISTER_MAG_OUT_X_H_M = 0x03,
    HMC5883_REGISTER_MAG_OUT_X_L_M = 0x04,
    HMC5883_REGISTER_MAG_OUT_Z_H_M = 0x05,
    HMC5883_REGISTER_MAG_OUT_Z_L_M = 0x06,
    HMC5883_REGISTER_MAG_OUT_Y_H_M = 0x07,
    HMC5883_REGISTER_MAG_OUT_Y_L_M = 0x08,
    HMC5883_REGISTER_MAG_SR_REG_Mg = 0x09,
    HMC5883_REGISTER_MAG_IRA_REG_M = 0x0A,
    HMC5883_REGISTER_MAG_IRB_REG_M = 0x0B,
    HMC5883_REGISTER_MAG_IRC_REG_M = 0x0C,
    HMC5883_REGISTER_MAG_TEMP_OUT_H_M = 0x31,
    HMC5883_REGISTER_MAG_TEMP_OUT_L_M = 0x32
} hmc5883MagRegisters_t;
/*MAGNETOMETER GAIN SETTINGS*/
typedef enum
{
    HMC5883_MAGGAIN_1_3 = 0x20,  // +/- 1.3
    HMC5883_MAGGAIN_1_9 = 0x40,  // +/- 1.9
    HMC5883_MAGGAIN_2_5 = 0x60,  // +/- 2.5
    HMC5883_MAGGAIN_4_0 = 0x80,  // +/- 4.0
    HMC5883_MAGGAIN_4_7 = 0xA0,  // +/- 4.7
    HMC5883_MAGGAIN_5_6 = 0xC0,  // +/- 5.6
    HMC5883_MAGGAIN_8_1 = 0xE0   // +/- 8.1
} hmc5883MagGain;
/*INTERNAL MAGNETOMETER DATA TYPE*/
typedef struct hmc5883MagData_s
{
    float x;
    float y;
    float z;
} hmc5883MagData;

/**
 * Class that contains sensors to be used to visually locate a block. This includes the Pixy and an IR sensor.
 */
class VisualSensor
{
  public:
    /**
     * Constructor. Intitialize the following variables
     * _pixy: Initialize the pixy camera.
     * _IRPort: Set the IRPort.
     * _stopVoltage: Set the stop voltage; The robot should stop whenever the
     *               input voltage from the IR sensor is greater than this voltage.
     */
    VisualSensor(const char IRPort, float stopVoltage);

    /**
     * Destructor
     */
    ~VisualSensor();

    /**
     * Find the correct block to go to.
     * Currently finds the lowest one in the Pixy's view (lower = closer).
     */
    Block getBlock();

    /**
     * Reads the input from the IRSensor port. This number is from 0-1023,
     * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is
     * in the voltage range we want.
     */
    boolean isClose();

    /**
     * Read the values from the IR sensor converted into 0.0 to 5.0 volts
     */
    float readProximity();

    void resetClosestBlock();

    Block badBlock; //Variable that is a "bad block", used when we find no good blocks
  private:
    Pixy _pixy; //Variable for pixy camera
    float _stopVoltage; //The robot should stop whenever the input voltage from the IR sensor is greater than this voltage.
    char _IRPort; //The port for the IR sensor
    Block closestBlock;
};

/**
 * The gryo allows you to get the current heading of the robot in degrees.
 */
class Gyro
{
public:
    //Constructor
    Gyro();

    //Destructor
    ~Gyro();

    /**
     * Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
     */
    float getDegrees();

    /**
     * Updates the current angle read by the gyro. Should be called every loop. Takes in the current time of the loop in millis().
     */
    void update(unsigned long currentTime);
private:
    L3G gyro;
    float angleZ;
    unsigned long previousTime;

    float averageBiasZ;
    float sigmaZ;
    float scaleFactorZ;
};

/**
 * The compass allows you to get the current heading in degrees and the initial heading of when the robot was first turned on.
 */
class Compass
{
public:	
    /**
     * Constructor. Set the initial heading whenever the program starts.
     * -declinationAngle: 'Error' of the magnetic field in your location. Find yours here: http://www.magnetic-declination.com/.
     * -(for Norman, I calculated it to be: 0.069522276053)
     */
    Compass(bool calibrate, float declinationAngle = 0.069522276053F);

    /**
     * Deconstructor
     */
    ~Compass();
    
    /**
     * Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
     */
    float getDegrees();

    /**
    * Set the magnetometer's gain.
    */
    void setGain(hmc5883MagGain gain);

private:
    hmc5883MagData _initMagVector; //Initial vector of the magnetic values read in when the robot is first started.
    hmc5883MagData _centerPoint; //center of the magnetic field when calibrated.
    float _rotationMatrix[3][3]; //Rotation matrix to rotate all points to the correct orientation
    bool _calibrate; //Wether or not we are calibrating the compass
    float _declinationAngle; // 'Error' of the magnetic field in your location. Find yours here: http://www.magnetic-declination.com/.
    
    float _hmc5883_Gauss_LSB_XY;  // Varies with gain
    float _hmc5883_Gauss_LSB_Z; // Varies with gain


    void calibrate();

    //Private Functions
    bool begin(void); //Set up the magnetometer
    void write8(byte address, byte reg, byte value); //Write data to the magnetometer

    /**
    * Returns the vector of magnetic values that the magnetometor is currently reading.
    * Bool adjusted determines whether or not to return the values adjusted for the center obtained during calibration.
    */
    hmc5883MagData getMagVector(bool adjusted);
};

#endif