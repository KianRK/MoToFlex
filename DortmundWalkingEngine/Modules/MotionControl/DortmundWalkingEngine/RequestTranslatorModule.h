#pragma once

#include "Representations/Modeling/Path.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Tools/Module/Module.h"
#include "RequestTranslator.h"
#include "Representations/MotionControl/WalkingInfo.h"

MODULE(RequestTranslatorModule)
	USES(WalkingInfo)
	REQUIRES(MotionSelection)
  REQUIRES(Path)
	PROVIDES_WITH_MODIFY(PatternGenRequest)
	PROVIDES(WalkingEngineParams)
	PROVIDES(FreeLegPhaseParams)
	PROVIDES(ControllerParams)
	PROVIDES(SpeedInfo)
END_MODULE

class RequestTranslatorModule : public RequestTranslatorModuleBase
{
public:
  RequestTranslatorModule();

protected:
  MotionSelection filteredMotionSelection;
	RequestTranslator translator;

	void update(PatternGenRequest& patternGenRequest);
	void update(WalkingEngineParams& walkingEngineParams);
	void update(ControllerParams& controllerParams);
	void update(SpeedInfo& speedInfo);
	void update(FreeLegPhaseParams& freeLegPhaseParams);
};

