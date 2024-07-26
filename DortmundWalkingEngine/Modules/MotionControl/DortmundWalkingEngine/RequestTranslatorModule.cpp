#include "RequestTranslatorModule.h"

RequestTranslatorModule::RequestTranslatorModule():
  filteredMotionSelection(),
  translator(filteredMotionSelection, thePath, theWalkingInfo)
{
}

void RequestTranslatorModule::update(PatternGenRequest& patternGenRequest)
{
	filteredMotionSelection = theMotionSelection;
if (theMotionSelection.targetMotion!=MotionRequest::walk || theMotionSelection.ratios[MotionRequest::walk] < 1.)
  {
	  filteredMotionSelection.walkRequest.pedantic=false;
	  filteredMotionSelection.walkRequest.speed.rotation=0.0;
	  filteredMotionSelection.walkRequest.speed.translation.x=0.0;
	  filteredMotionSelection.walkRequest.speed.translation.y=0.0;
	  filteredMotionSelection.walkRequest.target.rotation=0.0;
	  filteredMotionSelection.walkRequest.target.translation.x=0.0;
	  filteredMotionSelection.walkRequest.target.translation.y=0.0;
    filteredMotionSelection.walkRequest.forceOmniOnly = false;
  }
translator.updatePatternGenRequest(patternGenRequest);
}

void RequestTranslatorModule::update(WalkingEngineParams& walkingEngineParams)
{
	translator.updateWalkingEngineParams(walkingEngineParams);
}
void RequestTranslatorModule::update(FreeLegPhaseParams& freeLegPhaseParams)
{
	translator.updateFreeLegPhaseParams(freeLegPhaseParams);
}


void RequestTranslatorModule::update(ControllerParams& controllerParams)
{
	translator.updateControllerParams(controllerParams);
}

void RequestTranslatorModule::update(SpeedInfo& speedInfo)
{
	translator.updateSpeedInfo(speedInfo);
}

MAKE_MODULE(RequestTranslatorModule, Dortmund WalkingEngine) 
