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
* @file SwingLegController.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include <list>
#include "StepData.h"

#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Sensing/ZMPModel.h"

#ifndef WALKING_SIMULATOR
#include "Representations/Modeling/BallModel.h"
#else
#include "bhumanstub.h"
#endif

#define POLYNOM_DEGREE	4

using namespace std;

/**
 * @class SwingLegController
 * Calculates positions for the swing leg (not given by PatternGenerator)
 */
class SwingLegController
{
public:
	/** Constructor with all needed source data structures.
	 * @param theFootSteps Target positions of feet on the floor.
	 * @param theWalkingEngineParams Walking Engine Parameters.
	 */
	SwingLegController(
		const WalkingEngineParams	&theWalkingEngineParams,
		const FootSteps				&theFootSteps,
		const BallModel				&theBallModel,
		const MotionRequest			&theMotionRequest,
		const WalkingInfo			&theWalkingInfo,
		const FreeLegPhaseParams	&theFreeLegPhaseParams,
		const FallDownState			&theFallDownState,
		const ZMPModel				&theZMPModel):
		theWalkingEngineParams(theWalkingEngineParams),
		theFootSteps(theFootSteps),
    theBallModel(theBallModel),
		theMotionRequest(theMotionRequest),
		theWalkingInfo(theWalkingInfo),
		theFreeLegPhaseParams(theFreeLegPhaseParams),
		theFallDownState(theFallDownState),
		theZMPModel(theZMPModel)
	{ reset(); };

	/** Destructor */
	~SwingLegController(void) { freeMem(); };

	/** 
	 * Calculates the next position.
	 * @param footpositions Target data structure.
	 */
	void updateFootpositions(Footpositions &footpositions);

private:
	// Different phases when the robot stands on one leg:
	// During the starting phase we are in the single support phase
	// and freeLegBoundary is the last point of the leg on the floor.
	// During the ongoing phase the end is unknown, we are in a unlimitedSingleSupport.
	// During the ending phase we are in a single support, the upcomming double support
	// is planned and the freeLegBoundary is the next position of the free leg on the floor.
	// NA means we are not in a phase where we can use a leg.
	FreeLegPhase currentFreeLegPhase, kickPhase;

	const WalkingEngineParams	&theWalkingEngineParams;
	const FootSteps				&theFootSteps;
	const BallModel				&theBallModel;
	const MotionRequest			&theMotionRequest;
	const WalkingInfo			&theWalkingInfo;
	const FreeLegPhaseParams	&theFreeLegPhaseParams;
	const ZMPModel				&theZMPModel;
	const FallDownState			&theFallDownState; /**< Set by constructor */
	typedef list<Footposition *> FootList;

	/** List of all given target footpositions. */
	FootList footPositions;
	/** Pointer to the last position used for planning. */
	FootList::iterator lastPlannedReset[2];
	Point freeLegBoundary[2], *initWalkPol;
	int expectedBufferSize, additionalBuffer, freeLeg, kickCount, endTimestamp;
	/** Is the ZMP/IP-Controller running? */
	bool isRunning;
	bool stable;
	int stableCounter;
  int kickStartingCounter; // used to prevent staying forever in a waiting loop for stability before kicking
	/** How often to add the vectors until the position is reached */
	int goToStartCount, goToEndCount, endCounter;
	Vector3<double> goToStartVec, kickFootPos, currentVec, kickVec, kickStart, kickStop;
	/** Resets the controller. */
	void reset();
	/** 
	 * Deletes no more needed elements in footPositions. 
	 * @param footpositions Deleted positions contain the swing leg positions, so copy them here.
	 */
	void Shrink(Footpositions &footpositions);
	/** Free all memory. */
	void freeMem();

	/** 
	 * Add a foot position to the footPositions list.
	 * @param fp The list to add to.
	 */
	void addFootsteps(const Footposition &fp);
	void freeLegControl(int footNum);
	void initKick(int footNum);
	void initWalk(int footNum);
	void doKick(int footNum);
	/**
	 * Internal function to plan the next target position.
	 * @param footNum Number of foot (0 = left, 1 = right)
	 */
	void PlanFootReset(int footNum);
};

