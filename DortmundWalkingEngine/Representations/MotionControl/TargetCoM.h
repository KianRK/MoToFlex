/**
* @file TargetCoM 
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
* @class TargetCoM 
* Representing the target position of center of mass.
*/
class TargetCoM : public Streamable, public Point
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
	TargetCoM()
	{
	};

	/** Desctructor */
	~TargetCoM(){};
};
