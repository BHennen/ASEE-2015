#include "Sensors.h"


/**
 * Constructor. Intitialize the following variables
 * badBlock: Initialize all values of the block to -1.
 * _pixy: Initialize the pixy camera.
 * _IRPort: Set the IRPort.
 * _stopVoltage: Set the stop voltage; The robot should stop whenever the 
 *               input voltage from the IR sensor is greater than this voltage.
 */
VisualSensor::VisualSensor(const char IRPort, const float stopVoltage)
{
  //Set all the badBlock's values to -1
  badBlock.signature = -1;
  badBlock.x = -1;
  badBlock.y = -1;
  badBlock.width = -1;
  badBlock.height = -1;

  //Set all the closest blocks values
  closestBlock.y = 0;

  //Initialize pixy
  _pixy.init();

  //Set IR Port
  _IRPort = IRPort;
  _stopVoltage = stopVoltage;
}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{
}

/**
 * Find the correct block to go to. 
 * Currently finds the lowest one in the Pixy's view (lower = closer).
 * Returns null if there are no blocks found.
 */
Block VisualSensor::getBlock()
{
  //Get the number of blocks(detected objects) from the pixy
  int numBlocks = _pixy.getBlocks();

  if (numBlocks == 0)
  {
      return badBlock;
  }

  //Find the lowest block in the frame (which should be the closest block)
  //Higher y value means the block is lower in the frame.
  //Set the initial maximum to be the first block found.
  //int maxY = closestBlock.y;
  //Loop through each block
  for(int blockIndex = 0; blockIndex < numBlocks; blockIndex++)
  {
      //find maxY and index of that block
      if(_pixy.blocks[blockIndex].y >= closestBlock.y)
      {
          closestBlock = _pixy.blocks[blockIndex];
      }
  }
  return closestBlock;
}

void VisualSensor::resetClosestBlock()
{
    closestBlock.y = 0;
}

/**
 * Reads the input from the IRSensor port. This number is from 0-1023,
 * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is 
 * in the voltage range we want.
 */
boolean VisualSensor::isClose()
{
  float voltage = analogRead(_IRPort) * (5.0 / 1023.0);
  if (voltage > _stopVoltage)
  {
   return true;
  }
  return false;
}

/**
 * Read the value from the IRsensor port and converts it into a value from 0.0 - 5.0 volts.
 */
float VisualSensor::readProximity()
{
    float voltage = analogRead(_IRPort) * (5.0f / 1023.0f);
    return voltage;
}

//Constructor
Gyro::Gyro()
{
    Wire.begin();

    if(!gyro.init())
    {
        Serial.println("Failed to autodetect gyro type!");
        while(1);
    }
    gyro.enableDefault();

    previousTime = 0UL;
    angleZ = 0.0f;

    //Read values from eeprom
    //Read averageBiasZ
    int nextAddress = EEPROM_readAnything(0, averageBiasZ);
    //read sigmaZ
    nextAddress += EEPROM_readAnything(nextAddress, sigmaZ);
    //read scaleFactor
    nextAddress += EEPROM_readAnything(nextAddress, scaleFactorZ);

    Serial.print("Bias: ");
    Serial.print(averageBiasZ);
    Serial.print("\tSigma: ");
    Serial.print(sigmaZ);
    Serial.print("\tScale Factor: ");
    Serial.println(scaleFactorZ);
}

//Destructor
Gyro::~Gyro()
{

}

/**
* Returns the current heading of the robot in degrees, based on the initial heading of the robot. (0 <= degrees < 360)
*/
float Gyro::getDegrees()
{
    return angleZ;
}

/**
* Updates the current angle read by the gyro. Should be called every loop. Takes in the current time of the loop in millis().
*/
void Gyro::update(unsigned long currentTime)
{
    gyro.read();

    unsigned long sampleTime = currentTime - previousTime;
    previousTime = currentTime;

    //find current rate of rotation
    float rateZ = ((float) gyro.g.z - averageBiasZ);
    if(abs(rateZ) < 3 * sigmaZ)
    {
        rateZ = 0.0f;
    }

    //find angle
    angleZ += (rateZ * sampleTime / 1000.0f) * scaleFactorZ; //divide by 1000(convert to sec)

    // Keep our angle between 0-359 degrees
    if(angleZ < 0) angleZ += 360;
    else if(angleZ >= 360) angleZ -= 360;

    // Serial.print("Angle = "); Serial.print(angleZ); Serial.print("\tRate = "); Serial.println(rateZ * scaleFactorZ);
}
    

/**
* Constructor. Set the initial heading whenever the program starts.
* -declinationAngle: 'Error' of the magnetic field in your location. Find yours here: http://www.magnetic-declination.com/.
* -(for Norman, I calculated it to be: 0.069522276053)
*/
Compass::Compass(bool calibrate, float declinationAngle)
{
    /* Initialize the sensor */
    if(!begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    }

    _hmc5883_Gauss_LSB_XY = 1100.0F;  // Varies with gain
    _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

    _calibrate = calibrate;
    _declinationAngle = declinationAngle;

    if(_calibrate)
    {
        _centerPoint.x = 0.0;
        _centerPoint.y = 0.0;
        _centerPoint.z = 0.0;

        this->calibrate();
    }
    
    //Read values from eeprom
    int nextAddress = 0;
    nextAddress = EEPROM_readAnything(nextAddress, _centerPoint) + 1;
    EEPROM_readAnything(nextAddress, _rotationMatrix);
    _initMagVector = getMagVector(true);
    
}

/**
* Deconstructor
*/
Compass::~Compass()
{

}

void printVect(float v[3])
{
    Serial.print("X=");	Serial.print(v[0]);
    Serial.print(" Y="); Serial.print(v[1]);
    Serial.print(" Z="); Serial.println(v[2]);
}

void printMatrix(float m[3][3])
{
    Serial.print("|"); Serial.print(m[0][0]); Serial.print(" "); Serial.print(m[0][1]); Serial.print(" "); Serial.print(m[0][2]); Serial.println("|");
    Serial.print("|"); Serial.print(m[1][0]); Serial.print(" "); Serial.print(m[1][1]); Serial.print(" "); Serial.print(m[1][2]); Serial.println("|");
    Serial.print("|"); Serial.print(m[2][0]); Serial.print(" "); Serial.print(m[2][1]); Serial.print(" "); Serial.print(m[2][2]); Serial.println("|");
}

//Return distance between two points
float distance(hmc5883MagData p1, hmc5883MagData p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}

/**
* Rotates a given point (inPoint) about a given a rotation matrix (matrix) and stores the result in outPoint
* Has the option to calculate the rotation of Z by setting calculateZ to true
*/
void rotate(float inPoint[3], float matrix[3][3], float outPoint[3], bool calculateZ)
{
    outPoint[0] = matrix[0][0] * inPoint[0] + matrix[0][1] * inPoint[1] + matrix[0][2] * inPoint[2];
    outPoint[1] = matrix[1][0] * inPoint[0] + matrix[1][1] * inPoint[1] + matrix[1][2] * inPoint[2];
    if(calculateZ)
    {
        outPoint[2] = matrix[2][0] * inPoint[0] + matrix[2][1] * inPoint[1] + matrix[2][2] * inPoint[2];
    }
}

/**
* Multiply 2 3x3 matrices (inMatrix1 and inMatrix2) and stores the result in outMatrix
*/
void matrixMultiply3x3(float inMatrix1[3][3], float inMatrix2[3][3], float outMatrix[3][3])
{
    for(int i = 0; i<3; i++)
    {
        for(int j = 0; j<3; j++)
        {
            outMatrix[i][j] = 0;
            for(int k = 0; k<3; k++)
            {
                outMatrix[i][j] += inMatrix1[i][k] * inMatrix2[k][j];
            }
        }
    }
}

/**
* Multiply a matrix times a scalar
*/
void matrixTimesScalar(float inMatrix[3][3], float scalar, float outMatrix[3][3])
{
    for(int i = 0; i<3; i++)
    {
        for(int j = 0; j<3; j++)
        {
            outMatrix[i][j] = inMatrix[i][j] * scalar;
        }
    }
}

/**
* Add 2 3x3 matrices inMatrix1 and inMatrix2 and stores the result in outMatrix
*/
void addMatrices(float inMatrix1[3][3], float inMatrix2[3][3], float outMatrix[3][3])
{
    for(int i = 0; i<3; i++)
    {
        for(int j = 0; j<3; j++)
        {
            outMatrix[i][j] = inMatrix1[i][j] + inMatrix2[i][j];
        }
    }
}

/**
* Finds the rotation matrix between two points (p1 and p2) and stores the result in outRotationMatrix
* Uses rodrigues roation formula. Set axis to {0,0,0} if you dont care what axis(axes) the points are rotated about.
*/
void findRotationMatrix(float p1[3], float p2[3], float outRotationMatrix[3][3], float axis[3])
{
    //Calculate magnitude of desired axis
    float magAxis = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    float rotAxis[3] = {axis[0], axis[1], axis[2]};
    //If magAxis is not 0, we want to rotate about a certain axis. Make sure it is normalized.
    if(magAxis != 0)
    {
        rotAxis[0] /= magAxis;
        rotAxis[1] /= magAxis;
        rotAxis[2] /= magAxis;
    }

    //Calculate magnitude of the two points
    float magP1 = sqrt(p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2]);
    float magP2 = sqrt(p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2]);

    //Calculate normalized vectors a and b.
    float a[3] = {
        (p1[0] / magP1),
        (p1[1] / magP1),
        (p1[2] / magP1)};
    float b[3] = {
        (p2[0] / magP2),
        (p2[1] / magP2),
        (p2[2] / magP2)};
    //Subtract from them the normalized desired axis of rotation
    a[0] *= (1 - rotAxis[0]);
    a[1] *= (1 - rotAxis[1]);
    a[2] *= (1 - rotAxis[2]);
    b[0] *= (1 - rotAxis[0]);
    b[1] *= (1 - rotAxis[1]);
    b[2] *= (1 - rotAxis[2]);
    //Renormalize vectors
    float magA = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float magB = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    a[0] /= magA;
    a[1] /= magA;
    a[2] /= magA;
    b[0] /= magB;
    b[1] /= magB;
    b[2] /= magB;

    //calculate cross product of a and b (a x b = v)
    float v[3] = {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };

    //Sine of angle (magnitude of v)
    float s = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    //cosine of angle (dot product of a and b)
    float c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    //Calculate the axis of rotation 
    float vNorm[3];
    if(magAxis == 0)
    {//If no rotation axis was given,
        //calculate cross product normalized
        if(s != 0)
        {//only if the cross product exists, to prevent division by 0
            vNorm[0] = v[0] / s;
            vNorm[1] = v[1] / s;
            vNorm[2] = v[2] / s;
        }
        else
        {//v is 0
            vNorm[0] = v[0];
            vNorm[1] = v[1];
            vNorm[2] = v[2];
        }
    }
    else
    {//Set axis of rotation to be the given normalized axis. The sign of the axis is determined by the sign of the cross product
        vNorm[0] = (v[0] > 0) ? rotAxis[0] : -rotAxis[0];
        vNorm[1] = (v[1] > 0) ? rotAxis[1] : -rotAxis[1];
        vNorm[2] = (v[2] > 0) ? rotAxis[2] : -rotAxis[2];
    }

    //Identity Matrix = I
    float identity[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };
    //cosine times I = cI
    float cI[3][3];
    matrixTimesScalar(identity, c, cI);

    //skew-symmetric cross-product matrix = K
    float K[3][3] = {
            {0, -vNorm[2], vNorm[1]},
            {vNorm[2], 0, -vNorm[0]},
            {-vNorm[1], vNorm[0], 0}
    };
    //sine times K = sK 
    float sK[3][3];
    matrixTimesScalar(K, s, sK);

    //Tensor product = T
    float T[3][3] = {
            {vNorm[0] * vNorm[0], vNorm[0] * vNorm[1], vNorm[0] * vNorm[2]},
            {vNorm[0] * vNorm[1], vNorm[1] * vNorm[1], vNorm[1] * vNorm[2]},
            {vNorm[0] * vNorm[2], vNorm[1] * vNorm[2], vNorm[2] * vNorm[2]},
    };
    //1 - cosine times T = (1 - c)*T
    float constTimesT[3][3];
    matrixTimesScalar(T, (1.0f - c), constTimesT);

    // cI + sK
    float cIPlussK[3][3];
    addMatrices(cI, sK, cIPlussK);

    //Rotation Matrix = (cI + sK) + (1 - c)*T
    addMatrices(cIPlussK, constTimesT, outRotationMatrix);

}

void Compass::calibrate()
{
    //Rotate the robot slowly in a circle (direction does not matter)
    //Calculate the center.x, center.y, and center.z by averaging all the values


    Serial.println("Start rotating the robot slowly when you see GO.");
    delay(1000);
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("GO");

    unsigned long timer = millis();
    hmc5883MagData startPoint = getMagVector(false);
    hmc5883MagData currentPoint = startPoint;
    hmc5883MagData prevPoint = startPoint;
    float distToPrevPoint = 999.0;

    int numPoints = 1;
    float xTotal = 0.0;
    float yTotal = 0.0;
    float zTotal = 0.0;

    hmc5883MagData southish = startPoint;
    float maxDistToStart = 0.0;
    hmc5883MagData westish = startPoint;
    float bestEquidistance = 0.0;

    while(_calibrate)
    {
        unsigned long currentTime = millis();
        float currDistToStart = distance(currentPoint, startPoint);
        
        //Only add value of the current point when it is sufficiently far away from the previous point. 
        //This will make the calibration data more uniform if there are pauses during rotation.
        if(distToPrevPoint > 1)
        {
            //Find the total x, y and z values of all the points
            xTotal += currentPoint.x;
            yTotal += currentPoint.y;
            zTotal += currentPoint.z;

            //Find the furthest point(southish) from the start point		
            if(currDistToStart > maxDistToStart)
            {
                maxDistToStart = currDistToStart;
                southish = currentPoint;
            }

            //Find the most equidistant point between the current point to start point and current point to southish point
            float currEquidistance = currDistToStart + distance(currentPoint, southish);
            if(currEquidistance > bestEquidistance)
            {
                bestEquidistance = currEquidistance;
                westish = currentPoint;
            }
            //Update prevpoint and numPoints since the current point was used in a calculation
            numPoints++;
            prevPoint = currentPoint;
        }

        //Get the next mag vector for the current point
        currentPoint = getMagVector(false);
        distToPrevPoint = distance(currentPoint, prevPoint);

        //When at least 5 seconds have passed...
        if(currentTime - timer >= 5000UL)
        {
            //...and the current point is close to the starting point, end the calibration. Calculate and save the center point to the EEPROM
            if(currDistToStart < 1)
            {
                //Find the center point
                _centerPoint.x = xTotal / numPoints;
                _centerPoint.y = yTotal / numPoints;
                _centerPoint.z = zTotal / numPoints;

                //Translate startPoint and westish point coordinates based on the new center
                startPoint.x -= _centerPoint.x;
                startPoint.y -= _centerPoint.y;
                startPoint.z -= _centerPoint.z;
                westish.x -= _centerPoint.x;
                westish.y -= _centerPoint.y;
                westish.z -= _centerPoint.z;
                //Convert to float[3] vectors
                float startPointVector[3] = {startPoint.x, startPoint.y, startPoint.z};
                float westishVector[3] = {westish.x, westish.y, westish.z};
                //Print the vectors' initial values
                Serial.println("StartPoint Vector Initially: ");
                printVect(startPointVector);
                Serial.println("Westish Vector Initially: ");
                printVect(westishVector);

                //Find rotation matrix from the start point to the positive Y axis (north).
                //After this rotation, startPoint should be on the +Y axis. {0,>0,0}
                float rotMatrixNorth[3][3];                
                float northVector[3] = {0, 1, 0};
                float rotAxisAny[3] = {0.0f, 0.0f, 0.0f}; //Don't care about rotation axis
                findRotationMatrix(startPointVector, northVector, rotMatrixNorth, rotAxisAny);
                //Print north vector after rotation to see if the rotation matrix was calculated successfully
                float startPointOnY[3];
                rotate(startPointVector, rotMatrixNorth, startPointOnY, true);
                Serial.println("StartPoint Vector after rotation to \"North\" {0,>0,0} : ");
                printVect(startPointOnY);
                
                //Rotate the westish point based on the north rotation matrix and store it in westishRot1
                float westishRot1[3];
                rotate(westishVector, rotMatrixNorth, westishRot1, true);
                //Find rotation matrix from westishRot1 to the negative X axis (West) on the XY plane
                //After this rotation, westishRot1 should be: {<0,Q,0} where Q is close to 0
                float westVector[3] = {-1, westishRot1[1], 0};
                float rotMatrixWest[3][3];
                float rotAxisY[3] = {0, 1, 0}; //Rotate about Y axis
                findRotationMatrix(westishRot1, westVector, rotMatrixWest, rotAxisY);
                //Print westish vector after rotation to see if the rotation matrix was calculated successfully
                float westishPointOnXY[3];
                rotate(westishRot1, rotMatrixWest, westishPointOnXY, true);
                Serial.println("Westish Vector after rotation to \"West\" {<0,Q,0} where Q is close to 0 : ");
                printVect(westishPointOnXY);
               
                //Calculate a single, combined rotation matrix of north rotation and west rotation and store it as a single rotation matrix
                //Composing matrices done in reverse order
                matrixMultiply3x3(rotMatrixWest, rotMatrixNorth, _rotationMatrix);
                //Print north and westish vector after combined rotation to see if the combined rotation matrix was calculated successfully
                float startPointOnYcombined[3];
                float westishPointOnXYcombined[3];
                rotate(startPointVector, _rotationMatrix, startPointOnYcombined, true);
                Serial.println("StartPoint Vector after combined rotation {0,>0,0} : ");
                printVect(startPointOnYcombined);
                rotate(westishVector, _rotationMatrix, westishPointOnXYcombined, true);
                Serial.println("Westish Vector after combined rotation {<0,Q,0} where Q is close to 0 : ");
                printVect(westishPointOnXYcombined);
                                
                //Write the center point to the eeprom
                int nextAddress = 0;
                nextAddress = EEPROM_writeAnything(nextAddress, _centerPoint) + 1;
                //Write the rotation matrix to eeprom
                EEPROM_writeAnything(nextAddress, _rotationMatrix);

                Serial.println("Data saved successfully!");

                _calibrate = false;
            }
        }
    }
}

/**
 * Returns the vector of magnetic values that the magnetometor is currently reading.
 * Bool adjusted determines whether or not to return the values adjusted for the center and rotation matrix obtained during calibration.
 */
hmc5883MagData Compass::getMagVector(bool adjusted)
{
    // Read the magnetometer
    Wire.beginTransmission((byte) HMC5883_ADDRESS_MAG);
    Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
    Wire.endTransmission();
    Wire.requestFrom((byte) HMC5883_ADDRESS_MAG, (byte) 6);

    // Wait around until enough data is available
    while(Wire.available() < 6);

    // Note high before low (different than accel)  
    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();

    // Shift values to create properly formed integer (low byte first)
    hmc5883MagData magData;
    magData.x = (int16_t) (xlo | ((int16_t) xhi << 8));
    magData.y = (int16_t) (ylo | ((int16_t) yhi << 8));
    magData.z = (int16_t) (zlo | ((int16_t) zhi << 8));

    // Convert values to correct numbers
    hmc5883MagData magVector;
    magVector.x = magData.x / _hmc5883_Gauss_LSB_XY * 100; //* 100 to convert from gauss to microtesla
    magVector.y = magData.y / _hmc5883_Gauss_LSB_XY * 100;
    magVector.z = magData.z / _hmc5883_Gauss_LSB_Z * 100;

    if(adjusted)
    {
        //Translate the values so they are centered
        magVector.x -= _centerPoint.x;
        magVector.y -= _centerPoint.y;
        magVector.z -= _centerPoint.z;
        
    }

    return magVector;
}

/**
* Returns the how many degrees the robot is rotated from the initial heading. Always positive, and always less than 180 degrees. (0 <= degrees < 180)
*/
float Compass::getDegrees()
{
    hmc5883MagData magVector = getMagVector(true); //The magnetic values stored in a vector
    float vectArray[3] = {magVector.x, magVector.y, magVector.z};
    float vectRot[3];
    rotate(vectArray, _rotationMatrix, vectRot, false);
    
    //Calculate current rotation
    float heading = atan2(vectRot[1], vectRot[0]);

    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if(heading > 2 * PI)
        heading -= 2 * PI;

    // Convert radians to degrees.
    float headingDegrees = heading * 180 / PI;
    return headingDegrees;
}

/**
 * Set the magnetometer's gain
 */
void Compass::setGain(hmc5883MagGain gain)
{
    write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte) gain);

    switch(gain)
    {
        case HMC5883_MAGGAIN_1_3:
            _hmc5883_Gauss_LSB_XY = 1100;
            _hmc5883_Gauss_LSB_Z = 980;
            break;
        case HMC5883_MAGGAIN_1_9:
            _hmc5883_Gauss_LSB_XY = 855;
            _hmc5883_Gauss_LSB_Z = 760;
            break;
        case HMC5883_MAGGAIN_2_5:
            _hmc5883_Gauss_LSB_XY = 670;
            _hmc5883_Gauss_LSB_Z = 600;
            break;
        case HMC5883_MAGGAIN_4_0:
            _hmc5883_Gauss_LSB_XY = 450;
            _hmc5883_Gauss_LSB_Z = 400;
            break;
        case HMC5883_MAGGAIN_4_7:
            _hmc5883_Gauss_LSB_XY = 400;
            _hmc5883_Gauss_LSB_Z = 255;
            break;
        case HMC5883_MAGGAIN_5_6:
            _hmc5883_Gauss_LSB_XY = 330;
            _hmc5883_Gauss_LSB_Z = 295;
            break;
        case HMC5883_MAGGAIN_8_1:
            _hmc5883_Gauss_LSB_XY = 230;
            _hmc5883_Gauss_LSB_Z = 205;
            break;
    }
}

/**
* Set up the magnetometer
*/
bool Compass::begin()
{
    // Enable I2C
    Wire.begin();

    // Enable the magnetometer
    write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

    // Set the gain to a known level
    setGain(HMC5883_MAGGAIN_1_3);

    return true;
}

/**
* Write data to the magnetometer
*/
void Compass::write8(byte address, byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write((uint8_t) reg);
    Wire.write((uint8_t) value);
    Wire.endTransmission();
}