/**
* @file PatternGenRequest.h
* Contains the request for the pattern genertor, e.g. the desired speed and the next walk state.
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#ifndef __PATTERNGENREQUEST_H__
#define __PATTERNGENREQUEST_H__
#ifndef WALKING_SIMULATOR
#include "Tools/Math/Pose2D.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif

/**
 * @class PatternGenRequest
 * Contains the request for the pattern genertor, e.g. the desired speed and the next walk state.
 */
class PatternGenRequest : public Streamable{
public:
	/** Constructor */
	PatternGenRequest()
	{
		newState=NA;
		speed.translation.x=0;
		speed.translation.y=0;
		speed.rotation=0;
    pitch = 0;
	};

	/** Desired pitch */
	double pitch;

	/** Desired speed */
	Pose2D speed;

	/** Desired walking state*/
	enum State
	{
		standby,		/**< Walking engine (ZMP/IP-Controller) not active. Hard coded foot positions used.*/
		ready,			/**< Set the desired height of center of mass, run ZMP/IP-Controller, but stand still. */
		walking,		/**< Walk by using the ZMP/IP-Controller */
		NA,				/**< Unknown request */
		emergencyStop,	/**< Stop NOW, e.g. in case of falling */
                standLeft,
                standRight,
		numOfStates
	};

	/** The requested new state */
	State newState;

	/** 
	 * Returns the name of a given state .
	 * @param state The state.
	 * @return The name of the string as string.
	 */
	static const char *getStateName(State state)
	{
		switch (state)
		{
		case standby: return "standby";
		case ready: return "ready";
		case walking: return "walking";
		case NA: return "NA";
		case emergencyStop: return "emergencyStop";
		case standLeft: return "standLeft";
		case standRight: return "standRight";
		case numOfStates: return "numOfStates";
		}
		return "";
	}


protected:
	virtual void serialize(In* in, Out* out)
	{  
		STREAM_REGISTER_BEGIN();
		STREAM(speed)
		STREAM_ENUM(newState, numOfStates, PatternGenRequest::getStateName);
		STREAM_REGISTER_FINISH();
	}

};

#endif
