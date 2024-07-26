/*
	Copyright 2011, Oliver Urbann
	All rights reserved.

	This file is part of MoToFlex.

    MoToFlex is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MoToFlex is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

	Contact e-mail: oliver.urbann@tu-dortmund.de
*/

/**
* @file CSConverter.h
* 
* Converts the foot positions from world coordinate system to robot
* coordinate system by using the target center of mass in the world
* coordinate system and the actual center of mass in the robot 
* coordinate system.
*
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/

#pragma once

#include <list>
#include "StepData.h"


#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/FallDownState.h"


using namespace std;

/**
 * @class CSConverter
 * Converts the foot positions from world coordinate system to robot
 * coordinate system by using the target center of mass in the world
 * coordinate system and the actual center of mass in the robot 
 * coordinate system.
 */
class CSConverter
{
public:

	/** Constructor with all needed source data structures.
	 * @param theFootpositions New foot positions to add.
	 * @param theTargetCoM Actual target position of center of mass in world coordinate system.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 * @param theControllerParams Some controller parameters used here.
	 * @param theRobotModel Model of the robot. Contains the actual CoM.
	 * @param theFallDownState Information about the current state. If the robt has fallen down stop walking engine.
	 * @param theSensorData Data from sensors.
	 * @param theBodyTilt The current target roll and pitch of body.
	 */
	CSConverter(
		const Footpositions			&theFootpositions,
		const TargetCoM				&theTargetCoM,
		const WalkingEngineParams	&theWalkingEngineParams,
		const ControllerParams		&theControllerParams,
		const RobotModel			&theRobotModel,
		const FallDownState			&theFallDownState,
		const SensorData			&theSensorData,
		const BodyTilt				&theBodyTilt,
		const FreeLegPhaseParams	&theFreeLegPhaseParams);

	/** Destructor */
	~CSConverter(void);

	/** 
	 * Converts the foot positions from world coordinate system to robot
	 * coordinate system by using the target center of mass in the world
	 * coordinate system and the actual center of mass in the robot 
	 * coordinate system.
	 * @param kinematicRequest Filled with the foot positions in robot
	 * coordinate system and has to be sent to the inverse kinematics.
	 */
	void updateKinematicRequest(KinematicRequest &kinematicRequest);

	/**
	 * Updates some information about the current walk.
	 * @param walkingInfo Filled with some information about the walk.
	 */
	void updateWalkingInfo(WalkingInfo &walkingInfo);

private:
	const Footpositions			&theFootpositions; /**< Set by constructor */
	const TargetCoM				&theTargetCoM; /**< Set by constructor */
	const WalkingEngineParams	&theWalkingEngineParams; /**< Set by constructor */
	const ControllerParams		&theControllerParams; /**< Set by constructor */
	const RobotModel			&theRobotModel; /**< Set by constructor */
	const FallDownState			&theFallDownState; /**< Set by constructor */
	const SensorData			&theSensorData; /**< Set by constructor */
	const BodyTilt				&theBodyTilt; /**< Set by constructor */
	const FreeLegPhaseParams	&theFreeLegPhaseParams; /**< Set by constructor */

	typedef list<Footposition *> FootList;

	/** List of target foot positions in world coordinate system filled with the
		foot positions found in theFootpositions. No more needed positions
		are deleted in every step cycle. */	
	FootList footPositions;

	FootList::iterator currentFootpos; /**< Current target foot position in world coordinate system. */

	Point robotPosition; /**< Position of robot body in world coordinate system. */
	Point odometry; /**< Odometry data. */
	Point offsetToRobotPoseAfterPreview; /**< Future position of the robot when all steps in the preview are executed. */
	Point lastPos, 
		lastTargetCoM,
		lastSpeed, acc;

	bool fallingDown; /**< Is the robot falling? */
	bool lastFootPositionsValid;
	bool isLeavingPossible; /**< Is it possible to leave the walking engine without falling? */
	bool isRunning; /**< Is the walking engine currently running? */

	KinematicRequest lastRequest;

	/** Calculate the conversion with the given data
	 *	@param requiredOffset Filled with the foot positions in robot coordiaten system.
	 *  @param newCoMTarget Target position of the CoM in world coordinates.
	 *  @param curPos Target foot positions in world coordinates.
	 *  @param CoM Actual CoM in robot coordinate system.
	 */
	void toRobotCoords(StepData *requiredOffset, Point newCoMTarget, Footposition curPos, Point CoM);
	/** Resets the converter. */
	void reset();
	/** Deletes no more needed elements in footPositions. */
	void Shrink();
	/** Free all memory. */
	void freeMem();
	/** Add a foot position to the footPositions list */
	void addFootposition(const Footposition &fp);

};


