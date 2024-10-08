/** 
* @file MotionInfo.h
*
* Definition of class MotionInfo.
*
* @author Martin L�tzsch
*/

#ifndef __MotionInfo_h__
#define __MotionInfo_h__

#include "MotionRequest.h"
#include "Tools/Math/Pose2D.h"

/**
* @class MotionInfo
* Contains information about the motions which are executed by the Motion process.
*/
class MotionInfo : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM(executedMotionRequest);
    STREAM(isMotionStable);
    STREAM(positionInBremerWalkCycle);
    STREAM(positionInWalkCycle);
    STREAM(offsetToRobotPoseAfterPreview);
    STREAM_REGISTER_FINISH();
  }

public:
  /** 
  * Default constructor.
  */
  enum KickPhase { executing, done, NA };
  MotionInfo() : isMotionStable(false), positionInWalkCycle(0) {}
  
  MotionRequest executedMotionRequest; /**< The motion request that is currently executed. */
  bool isMotionStable; /**< If true, the motion is stable, leading to a valid camera matrix. */
  double positionInBremerWalkCycle; /**< The position inside the step of the Bremer walking engine slides from 0 to 1 during one step. */
  double positionInWalkCycle; 
  Pose2D offsetToRobotPoseAfterPreview;
  KickPhase kickPhase;
};

#endif // __MotionInfo_h__
