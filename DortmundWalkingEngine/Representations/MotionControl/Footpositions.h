/**
* @file Footpositions.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/** Maximum number of possible foot positions in buffer */
#define MAX_FP	300  

/**
 * @class Robot
 * Representation to transfer the target foot (including the swing leg) positions to other modules.
 */
class Footpositions : public Streamable
{
public :

	/** When the controller is not running this step is used. */
	StepData suggestedStep;

	/** Number of foot positions in buffer */
	int numOfFP;

	/** Is the controller running? */
	bool running;

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(numOfFP)
		STREAM_REGISTER_FINISH();
	};

	/** Constructor */
	Footpositions()
	{
		numOfFP = 0;
	};

	/** Desctructor */
	~Footpositions(){};

	/** Initialize the data. */
	void init()
	{
		numOfFP=0;
	}
	
	/**
	 * Adds a foot position to the buffer.
	 * \param fp Foot position to add.
	 */
	void addFP(Footposition &fp)
	{
		if (numOfFP>MAX_FP)
		{
			printf("addFP(): Overflow");
			return;
		}
		this->fp[numOfFP]=fp;
		numOfFP++;
	}

	Footposition getFP(int i) const
	{
		if (i>MAX_FP)
		{
			printf("getFP(): Index out of range");
			return fp[0];
		}
		return fp[i];
	}
private:
	/** 
	 *	Buffer for target foot positions. There might be more than one position
	 *	per frame to fill the preview buffer needed by the preview controller
	 *	ZMP/IP-Controller 
	 */
	Footposition fp[MAX_FP];

};
