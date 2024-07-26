/**
* @file WalkingInformations.h
* This file contains some structures used by the walking engine to describe the walk.
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once
#define MAX_PARAM_FILENAME_LEN 1000
#include "Point.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cstring>

struct WalkingParams
{
	unsigned int 
		crouchingDownPhaseLen, /**< Duration for the crouching down phase */
		stoppingPhaseLen, /**< Duration for the stopping phase */
		startingPhaseLen; /**< Duration for the starting phase */

	double
		doubleSupportRatio; /**< The double support ration */

};

/** Current state of PatternGenerator. */
enum State
{
	standby,		/**< Standing, no ZMP/IP-Controller needed. */
	ready,			/**< Standing, ZMP/IP-Controller running and ready to go. */
	walking,		/**< Walking using ZMP/IP-Controller. */
	stopping,		/**< Stopping but still walking. */
	goingToReady,	/**< Going to standing with ZMP/IP-Controller. */
	goingToStandby, /**< Going to standing without ZMP/IP-Controller. */
	NA				/**< Unknown state. */
};

enum StandType
{
	doubleSupport,
	leftSingleSupport,
	rightSingleSupport,
	numOfStandTypes
};

/** Current walking phase. */
enum WalkingPhase
{
	firstSingleSupport,		/**< Standing on left foot. */
	firstDoubleSupport,		/**< Phase after firstSingleSupport. */
	secondSingleSupport,	/**< Standing on right foot. */
	secondDoubleSupport,	/**< Phase after secondSingleSupport. */
	unlimitedDoubleSupport, /**< Double support with unknown end. */
	unlimitedSingleSupport, /**< Single support with unknown end. */
        unlimitedSingleSupportLeft,
	unlimitedSingleSupportRight,
	numOfWalkingPhases
};

enum FreeLegPhase { starting, ongoing, ending, freeLegNA, numOfFreeLegPhases };

class MovementInformation
{
public:
	Point speed; /**< The new speed (x, y translation and rotation) */
	Point destination; /**< The new destination. */
	/**
	 * Equality check.
	 * @param other The other MovementInformation.
	 * @return true, if the instances are equal, false otherwise.
	 */
	bool operator != (MovementInformation other)
	{
		return speed!=other.speed || destination!=other.destination;
	}
};

class RobotInformation
{
public:
	/** destructor */
	RobotInformation()
	{
		std::strcpy(paramFile, "parameters.dat");
	}

	/** 
	* Sets the path to the parameter file
	* @param path Path to the file
	* @return false, if failed
	*/
	bool setParamFile(char *path)
	{
		strncpy(paramFile, path, MAX_PARAM_FILENAME_LEN);
		return true;
	}

	char paramFile[MAX_PARAM_FILENAME_LEN]; /**< The path to the parameter file */
};


