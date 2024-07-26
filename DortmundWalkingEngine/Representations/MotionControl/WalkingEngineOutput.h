/**
* @file Representations/MotionControl/WalkingEngineOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#ifndef __WalkingEngineOutput_H__
#define __WalkingEngineOutput_H__

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"

/**
* @class WalkingEngineOutput
* A class that represents the output of the walking engine.
*/
class WalkingEngineOutput : public JointRequest
{
protected:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM_BASE(JointRequest);
    STREAM(speed);
    STREAM(odometryOffset);
    STREAM(offsetToRobotPoseAfterPreview);
    STREAM(isLeavingPossible);
    STREAM(positionInWalkCycle);
    STREAM_REGISTER_FINISH();
  }

public:
  Pose2D speed, /**< The current walking speed. */
         odometryOffset; /**< The body motion performed in this step. */
	Pose2D offsetToRobotPoseAfterPreview; /**< The offset from current pose to the pose after execution of any buffered motion commands (i.e. walking preview or similar) */
  bool isLeavingPossible; /**< Is leaving the motion module possible now? */
  double positionInWalkCycle; /**< The current position in the walk cycle in the range [0..1[. */

  /** 
  * Default constructor.
  */
  WalkingEngineOutput() : isLeavingPossible(true), positionInWalkCycle(0){}
};

#endif // __WalkingEngineOutput_H__
