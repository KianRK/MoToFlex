/**
* @file CoMProvider.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once
#include <list>
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Sensing/RobotModel.h"
#include "StepData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/ActualCoM.h"

using namespace std;

/**
 * @class CoMProvider
 * Determines the target orientation of the body.
 */


class CoMProvider
{
public:
	/** Constructor with all needed source data structures.
	 * @param theSensorData Measured data.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
	CoMProvider(
		const JointData				&theJointData,
		const WalkingEngineParams	&theWalkingEngineParams,
		const JointRequest			&theJointRequest,
		const FootSteps				&theFootSteps,
		const RobotModel			&theRobotModel);

	/** Destructor */
	~CoMProvider(void);

	/** 
	 * Calculates the next target orientation.
	 * @param bodyTilt Target data structure.
	 */
	void updateActualCoM(ActualCoM &theActualCoM);

private:
		const JointData				&theJointData;
		const WalkingEngineParams	&theWalkingEngineParams;
		const JointRequest			&theJointRequest;			/**< Set by constructor. */
		const FootSteps				&theFootSteps;
		const RobotModel			&theRobotModel;

		typedef list<Footposition *> FootList;

		/** List of target foot positions in world coordinate system filled with the
			foot positions found in theFootpositions. No more needed positions
			are deleted in every step cycle. */	
		FootList footPositions;
};

