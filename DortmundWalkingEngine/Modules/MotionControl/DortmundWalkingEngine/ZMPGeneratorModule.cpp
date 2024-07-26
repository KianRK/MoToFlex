/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPGeneratorModule.cpp
* Module wrapper for the ZMPGenerator
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#include "ZMPGeneratorModule.h"

ZMPGeneratorModule::ZMPGeneratorModule():
generator(
		theFootSteps,
		theWalkingEngineParams,
		theControllerParams,
		theFreeLegPhaseParams,
		theWalkingInfo)
{
}

void ZMPGeneratorModule::update(RefZMP &theRefZMP)
{
	generator.updateRefZMP(theRefZMP);
};


MAKE_MODULE(ZMPGeneratorModule, Dortmund WalkingEngine)