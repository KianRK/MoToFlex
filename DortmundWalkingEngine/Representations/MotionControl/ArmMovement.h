/**
* @file Representations/MotionControl/ArmMovement.h
* This file declares a class that represents the output of ArmAnimator.
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#ifndef __ArmMovement_H__
#define __ArmMovement_H__

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"

/**
* @class ArmMovement
* A class that represents the output of the walking engine.
*/
class ArmMovement : public JointData
{
protected:
	virtual void serialize(In* in, Out* out)
	{  
		STREAM_REGISTER_BEGIN();
		STREAM(usearms);
		STREAM_BASE(JointData);
		STREAM_REGISTER_FINISH();
	}

public:
	/** 
	* Default constructor.
	*/
	ArmMovement() : usearms(false) {}

	bool usearms;
};
#endif // __ArmMovement_H__
