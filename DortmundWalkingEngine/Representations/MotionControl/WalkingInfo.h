/**
* @file WalkingInfo.h
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Modules/MotionControl/DortmundWalkingEngine/WalkingInformations.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Math/Pose2D.h"
#include "Tools/Streams/Streamable.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif

/**
* @class WalkingInfo 
* Gives some information about the walk.
*/
class WalkingInfo : public Streamable{
	public:
		WalkingInfo() : isLeavingPossible(true), kickPhase(freeLegNA) {};

		Pose2D odometryOffset; /**< Distancte between last odometry position and current */
		Pose2D robotPosition; /**< Current position of body in world coordinate system of the walking engine */
		Pose2D offsetToRobotPoseAfterPreview; /**< Future position of robot after the preview phase */
		Vector2<double> expectedAcc; /**< Expected acceleration of the body */
		bool isLeavingPossible; /**< Is is possible to leave the walking engine without falling? */
		FreeLegPhase kickPhase;
		Vector2<double> ballCSinWEWCS;

	/** 
	  * The function returns names of the current kick phase.
	  * @param kickPhase The kick phase the name of which is returned.
	  * @return The corresponding name.
	 */
	static const char* getKickPhaseName(FreeLegPhase kickPhase)
	{
		switch(kickPhase)
		{
			case starting: return "starting";
			case ongoing: return"ongoing";
			case ending: return "ending";
			case freeLegNA: return "freeLegNA";
			default: return "unknown";
		}
	}

	protected:
		virtual void serialize(In* in, Out* out)
		{  
			STREAM_REGISTER_BEGIN();

			STREAM(odometryOffset);
			STREAM(robotPosition);
			STREAM(offsetToRobotPoseAfterPreview);
			STREAM(expectedAcc);
			STREAM(isLeavingPossible);
			STREAM_ENUM(kickPhase, numOfFreeLegPhases, WalkingInfo::getKickPhaseName);
			STREAM_REGISTER_FINISH();
		}
};




