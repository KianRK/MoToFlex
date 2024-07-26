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
 * @file dortmundwalkingengine.h
 * Used by the simulation to retrieve the target angles for the robot.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */


#pragma once	
#include "robot.h"
#include <string>

// Representations
#include "Representations/Modeling/FallDownState.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"

// Modules
#include "Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.h"
#include "Modules/MotionControl/DortmundWalkingEngine/ZMPGenerator.h"
#include "Modules/MotionControl/DortmundWalkingEngine/ZMPIPController.h"
#include "Modules/MotionControl/DortmundWalkingEngine/SwingLegController.h"
#include "Modules/MotionControl/DortmundWalkingEngine/CSConverter.h"
#include "Modules/MotionControl/DortmundWalkingEngine/TiltController.h"
#include "Modules/MotionControl/NaoKinematic.h"

/**
 * Used by the simulation to retrieve the target angles for the robot.
 * This class is an adapter to provide an uniform interface for all
 * possible ways to retrieve target angles. Here the Dortmund Walking Engine
 * is used to calculate the angles on line.
 */
class DortmundWalkingEngineAdapter
{
public:
	
	/** 
	 * Constructor.
	 */
	DortmundWalkingEngineAdapter();

	/** 
	 * Called by simulation.cpp to retrieve the target 
	 * angles for the next frame.
	 * \param angles array filled by getAngles with the target angles
	 * \param numOfAngles
	 */
	void getAngles(float angles[], int numOfAngles, Robot &robot);

	/**
	 * This function sets the path to the config files (without file
	 * names).
	 * \param path The path to the config files.
	 */
	void setSettingsPath(string path);

private:

	// Representations

	/** Target foot positions including the swing leg. */
	Footpositions theFootpositions;

	/** Target foot positions on ground (without swing leg) */
	FootSteps theFootSteps;

	/** Is the robot standing or is it lying on the floor? */
	FallDownState theFallDownState;

	/** The reference ZMP based on theFootSteps */
	RefZMP theRefZMP;

	/** All data measured by simulated sensors */
	SensorData theSensorData;

	/** Target position of the CoM */
	TargetCoM theTargetCoM;

	/** Target orientation of the body */
	BodyTilt theBodyTilt;

	/** Parameters for the ZMP/IP-Controller */
	ControllerParams theControllerParams;

	/** Global walking engine parameters */
	WalkingEngineParams theWalkingEngineParams;

	/** Some information about the walk */
	WalkingInfo theWalkingInfo;

	/** The measured ZMP */
	ZMPModel theZMPModel;

	/** Some information about the robot */
	RobotModel theRobotModel;

	/** The request which is used by the walking engine, e.g. the
		desired speed */
	PatternGenRequest thePatternGenRequest;

	/** The target angles */
	KinematicOutput theKinematicOutput;

	/** Target foot positions in robot coordinate system */
	KinematicRequest theKinematicRequest;

	/** Actual joint angles */
	JointData theJointData;

	/** Dimensions of the robot */
	RobotDimensions theRobotDimensions;

	/** Dummy */
	MotionRequest theMotionRequest;

	/** Dummy */
	FreeLegPhaseParams theFreeLegPhaseParams;

	/** Dummy */
	BallModel theBallModel;

	/** Dummy */
	JointCalibration theJointCalibration;

	/** Dummy */
	JointRequest theJointRequest;

	// Modules

	/** Generates the target foot steps */
	PatternGenerator patternGenerator;

	/** Calculates the reference ZMP based on the target foot steps*/
	ZMPGenerator zmpGenerator;

	/** Calculates the target CoM trajectory based on the
		reference ZMP */
	ZMPIPController zmpipController;

	/** Adds the positions of the swing leg to the foot steps */
	SwingLegController swingLegController;

	/** Calculates the foot positions in the robot coordinate system
		by using the target foot steps in the world coordinate system
		and the target CoM positions */
	CSConverter csConverter;

	/** Calculates the target body orientation */
	TiltController tiltController;

	/** The inverse kinematic */
	NaoKinematic naoKinematic;

	/**
	 * Loads the walking engine parameters
	 * \param the path to the parameter file
	 */
	void loadWEParams(const char *path);

	/**
	 * Loads the parameters for the ZMP/IP-Controller
	 * \param the path to the parameter file
	 */
	int loadCParams(const char *path);
};

