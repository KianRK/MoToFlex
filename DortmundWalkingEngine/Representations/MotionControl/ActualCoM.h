/**
* @file ActualCoM 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
* @class ActualCoM 
* Representing the actual position of center of mass in the walking engines
* world coordinate system.
*/
class ActualCoM : public Streamable, public Point
{
public :

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(x)
		STREAM(y)
		STREAM(z)
		STREAM_REGISTER_FINISH();
	};

	/** Constructor */
	ActualCoM()
	{
	};

	/** Desctructor */
	~ActualCoM(){};
};
