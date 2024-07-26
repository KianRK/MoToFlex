/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CSConverterModule.h
* Module wrapper for the CSConverter
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/FallDownState.h"

#include "CSConverter.h"

MODULE(CSConverterModule)
	REQUIRES(TargetCoM)
	REQUIRES(WalkingEngineParams)
	REQUIRES(FreeLegPhaseParams)
	REQUIRES(ControllerParams)
	REQUIRES(RobotModel)
	REQUIRES(BodyTilt)
	REQUIRES(SensorData)
	REQUIRES(FallDownState)
	REQUIRES(Footpositions)
	PROVIDES_WITH_MODIFY(KinematicRequest)
	PROVIDES_WITH_MODIFY(WalkingInfo)
END_MODULE

class CSConverterModule : public CSConverterModuleBase
{
public:
	CSConverterModule();

	CSConverter converter;

	void update(KinematicRequest &kinematicRequest);
	void update(WalkingInfo &walkingInfo);
};



