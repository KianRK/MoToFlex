/**
* @class WalkingInfo 
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Streams/Streamable.h"
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Modules/MotionControl/DortmundWalkingEngine/WalkingInformations.h"

class FreeLegInfo : public Streamable{
	public:
		FreeLegInfo() : isLeavingPossible(true) {};

		Pose2D odometryOffset;
		Pose2D robotPosition;
		Pose2D offsetToRobotPoseAfterPreview;
		Vector2<double> expectedAcc;
		bool isLeavingPossible;
		FreeLegPhase kickPhase;
		Vector2<double> ballCSinWEWCS;

	protected:
		virtual void serialize(In* in, Out* out)
		{  
			STREAM_REGISTER_BEGIN();

			STREAM(odometryOffset);
			STREAM(robotPosition);
			STREAM(offsetToRobotPoseAfterPreview);
			STREAM(expectedAcc);
			STREAM(isLeavingPossible);
			STREAM_REGISTER_FINISH();
		}
};




