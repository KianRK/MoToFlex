/**
* @file SpeedInfo 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once
#include "Tools/Streams/Streamable.h"
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"

/**
 * @class SpeedInfo
 * Contains information about the actual speed.
 */
class SpeedInfo : public Streamable
{
public :

	Point speed; /**< The actual speed. */

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM_REGISTER_FINISH();
	};

	/** Constructor */
	SpeedInfo(){};

	/** Destructor */
	~SpeedInfo(){};
};
