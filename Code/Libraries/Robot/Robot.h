#ifndef ROBOT_H
#define ROBOT_H

#include <Drivetrain.h>
#include <Sensors.h>
#include <FishManager.h>

class ASEE2015
{
public:
	/**
	 * Construct a new asee robot!
	 * testParam: A parameter for testing various modules of the robot. Enters a different mode based on the value of this parameter.
	 *****************************************************************************************************************************
	 * testParam | Description                                                                  | Modules required               *
	 *___________|______________________________________________________________________________|________________________________*
	 *     1     | Final test of the robot! This is the final product. Goes around the track and| All                            *
	 *           | picks up all the fish, storing them inside. Dumps them all off too.          |                                *
	 *___________|______________________________________________________________________________|________________________________*
	 *     2     | Test PID and gyro. Goes to closest fish seen and stops in front of it.       | eyes, wheels, gyro             *
	 *___________|______________________________________________________________________________|________________________________*
	 *     3     | Test PID and Conveyor. Goes to closest fish seen, picks it up, and stores it | eyes, wheels, conveyor         *
	 *           | based on its color                                                           |                                *
	 *___________|______________________________________________________________________________|________________________________*
	 *     4     | Test Conveyor. Picks up a fish and stores it. Has a pattern:                 | conveyor                       *
	 *           | 1,2,3,1, 1,2,3,2, 1,2,3,3 ...                                                |                                *
	 *___________|______________________________________________________________________________|________________________________*
	 *     5     | Test fishCollection. Goes to closest fish seen, picks it up, and stores it   | eyes, wheels, conveyor, gyro   *
	 *           | based on its color. Then rotates to next fish and repeats for all the fish.  |                                *
	 *___________|______________________________________________________________________________|________________________________*
	 *     6     | Test Conveyor. Picks up a fish, determines color using pixy, and stores it.  | conveyor, eyes                 *
	 *___________|______________________________________________________________________________|________________________________*
	 *     7     | Test PID. Goes to closest fish seen and stops in front of it, rotates to next| eyes, wheels, gyro             *
	 *           | fish seen until all 12 fish have been visited.                               |                                *
	 *___________|______________________________________________________________________________|________________________________*
	 */
	ASEE2015(int testParam, Drivetrain* driveTrain, VisualSensor* visualSensor, Gyro* gyro, Conveyor* conveyor, Bins* bins);
	~ASEE2015();

	/**
	 * Tell the robot to execute a function based on which modules it has and the value of testParam.
	 * Returns true when it has finished the test.
	 */
	boolean go();

	/**
	 * Stops all the motors for all the modules
	 */
	void allStop();
private:
	//Pointers to optional modules for the ASEE robot.
	Drivetrain* _wheels;
	VisualSensor* _eyes;
	Gyro* _gyro;
	Conveyor* _conveyor;
	Bins* _bins;
	boolean _readyToGo; //whether or not the robot is ready to go
	int mode; //What mode the robot is in base on testParam

	int _fishSignature; //The fish signature that the getblock method saw most often before we pick up the fish

	boolean goFinalRun(unsigned long currentTime);

	/**
	* Uses the PID, pixy, and IR sensor to go to the closest fish and stop in front of it. Returns true when
	* the IR sensor is close to something. If the pixy doesnt see a fish, it will stop and return false.
	*/
	boolean goToFishAndStop(unsigned long currentTime);

	boolean goToBinAndStop(unsigned long currentTime);

	/**
	* Travels around the track collecting all the fish.
	* Returns true when it has done so.
	*/
	boolean collectFish(unsigned long currentTime);

	/**
	 * Tests the conveyor code infinitely. Has the option of using the pixy to determine what bin to store the fish in,
	 * otherwise it goes in a set pattern.
	 */
	void testConveyor(unsigned long currentTime, boolean usePixy);

	/**
	* Travels around the track stopping in front of all the fish.
	* Returns true when it has done so.
	*/
	boolean testPIDRotate(unsigned long currentTime);

	/**
	* Goes to the each of the bins, repositions, then dumps the fish
	*/
	boolean dumpFish(unsigned long currentTime);

	void testConveyorRotate(unsigned long currentTime);

	boolean testBinDumping(unsigned long currentTime);

	boolean setup(unsigned long currentTime);
};

#endif