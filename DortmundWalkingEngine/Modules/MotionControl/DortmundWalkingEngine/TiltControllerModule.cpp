/** 
* @file Modules/MotionControl/DortmundWalkingEngine/TiltControllerModule.cpp
* Module wrapper for the TiltController
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#include "TiltControllerModule.h"

TiltControllerModule::TiltControllerModule():
controller(
		theSensorData,
		theWalkingEngineParams,
		theJointCalibration,
		theJointRequest)
{
}

void TiltControllerModule::update(BodyTilt &theBodyTilt)
{
	controller.updateBodyTilt(theBodyTilt);
};


MAKE_MODULE(TiltControllerModule, Dortmund WalkingEngine)