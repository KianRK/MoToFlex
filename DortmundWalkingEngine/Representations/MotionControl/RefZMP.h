/**
* @class RefZMP 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Platform/GTAssert.h"
#else
#include "bhumanstub.h"
#endif

/** Maximum number of possible foot positions in buffer */
#define MAX_ZMP	300

/**
 * @class RefZMP
 * Representation to transfer the reference ZMP.
 */
class RefZMP : public Streamable
{
public :
	int numOfZMP;	  /**< Number of ZMP point stored in the buffer */
	bool running;	  /**< Is the controller running? */

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(numOfZMP)
		STREAM(running)
		STREAM_REGISTER_FINISH();
	};

	/** Constructor */
	RefZMP(){ numOfZMP=0; };

	/** Desctructor */
	~RefZMP(){};

	/** Initialize the data. */
	void init()
	{
		numOfZMP=0;
	}
	
	/**
	 * Adds a reference ZMP to the buffer.
	 * \param newzmp Reference ZMP to add.
	 */
	void addZMP(ZMP newzmp)
	{
		if (numOfZMP>MAX_ZMP)
		{
			printf("addZMP(): Overflow");
			return;
		}
		zmp[numOfZMP]=newzmp;
		numOfZMP++;
	}

	ZMP getZMP(int i) const
	{
		if (i>MAX_ZMP)
		{
			printf("getZMP(): Index out of range");
			return zmp[0];
		}
		return zmp[i];
	}

private:
	ZMP zmp[MAX_ZMP]; /**< Buffer with reference ZMPs. */
};
