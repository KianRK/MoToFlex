/**
* @file KickInfo.h
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Modules/MotionControl/DortmundWalkingEngine/WalkingInformations.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
* @class WalkingInfo 
* Gives some information about the walk.
*/
class KickInfo : public Streamable{
public:
	KickInfo() : kickLeftPossible(false),  kickRightPossible(true) {};
	bool kickLeftPossible, kickRightPossible;
protected:
	virtual void serialize(In* in, Out* out)
	{  
		STREAM_REGISTER_BEGIN();
		STREAM(kickLeftPossible);
		STREAM(kickRightPossible);
		STREAM_REGISTER_FINISH();
	}
};




