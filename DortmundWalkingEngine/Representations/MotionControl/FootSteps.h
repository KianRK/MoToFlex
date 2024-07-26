/**
* @class FootSteps 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#ifndef _FOOTSTEPS_H
#define _FOOTSTEPS_H

#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/** Maximum number of possible foot steps in buffer */
#define MAX_STEPS	300

/**
 * @class Robot
 * Representation to transfer the target foot steps to other modules.
 */
class FootSteps : public Streamable
{
public :

	/** When the controller is not running this step is used. */
	StepData suggestedStep;

	/** Number of foot positions in buffer */
	int numOfSteps;

	/** Is the controller running? */
	bool running;

	/** Immediate stop in case of emergency. */
	bool emergencyStop;

	/** Target pitch. Obsolete. */
	double pitch;

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(numOfSteps)
		STREAM(running)
		STREAM(pitch)
		STREAM_REGISTER_FINISH();
	};

	/** Constructor */
	FootSteps()
	{
		running=false;
		emergencyStop=false;
		numOfSteps = 0;
	};

	/** Initialize the data. */
	void reset()
	{
		running=false;
		emergencyStop=false;
		numOfSteps = 0;
	}

	/** Destructor */
	~FootSteps(){};

	void addStep(Footposition newstep)
	{
		if (numOfSteps>MAX_STEPS)
		{
			printf("addStep(): Overflow");
			return;
		}
		steps[numOfSteps]=newstep;
		numOfSteps++;
	}

	Footposition getStep(int i) const
	{
		if (i>MAX_STEPS)
		{
			printf("getStep(): Index out of range");
			return steps[0];
		}
		return steps[i];
	}

private:
	
	/** 
	 *	Buffer for target foot steps. There might be more than one step
	 *	per frame to fill the preview buffer needed by the preview controller
	 *	ZMP/IP-Controller 
	 */
	Footposition steps[MAX_STEPS];
};
#endif
