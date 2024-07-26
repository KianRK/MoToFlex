/**
* @class BufferedData 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#else
#include "StepData.h"
#include "bhumanstub.h"
#endif


/**
 * @class BufferedData
 * Base class for alle representations using buffered data like foot steps or reference ZMPs.
 */
template <class T> BufferedData
{
public :
	ZMP zmp[MAX_BUFFER]; /**< Buffer with reference ZMPs. */
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
		zmp[numOfZMP]=newzmp;
		numOfZMP++;
	}
};
