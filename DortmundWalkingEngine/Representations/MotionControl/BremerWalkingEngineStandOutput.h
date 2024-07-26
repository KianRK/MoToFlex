/**
* @file Representations/MotionControl/WalkingEngineStandOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#ifndef __BremerStandOutput_H__
#define __BremerStandOutput_H__

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"

/**
* @class BremerWalkingEngineOutput
* A class that represents the output of the bremer walking engine.
*/
class BremerWalkingEngineStandOutput : public JointRequest
{
protected:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM_BASE(JointRequest);
    STREAM_REGISTER_FINISH();
  }

public:
  /** 
  * Default constructor.
  */
  BremerWalkingEngineStandOutput() {}
};

#endif // __StandOutput_H__
