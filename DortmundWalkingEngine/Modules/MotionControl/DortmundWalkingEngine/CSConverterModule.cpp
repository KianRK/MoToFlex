/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CSConverterModule.cpp
* Module wrapper for the CSConverter
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#include "CSConverterModule.h"

CSConverterModule::CSConverterModule():
converter(
		theFootpositions,
		theTargetCoM,
		theWalkingEngineParams,
		theControllerParams,
		theRobotModel,
		theFallDownState,
		theSensorData,
		theBodyTilt,
		theFreeLegPhaseParams)
{
}

void CSConverterModule::update(KinematicRequest &kinematicRequest)
{
	converter.updateKinematicRequest(kinematicRequest);
};

void CSConverterModule::update(WalkingInfo &walkingInfo)
{
	converter.updateWalkingInfo(walkingInfo);
};

MAKE_MODULE(CSConverterModule, Dortmund WalkingEngine)