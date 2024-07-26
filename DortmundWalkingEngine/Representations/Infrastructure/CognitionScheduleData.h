/**
* @file CognitionScheduleData.h
* 
* Declaration of class CognitionScheduleData
*/ 

#ifndef __CognitionScheduleData_h_
#define __CognitionScheduleData_h_

#include "Tools/Streams/Streamable.h"

/**
* The class represents the states of the keys.
*/
class CognitionScheduleData : public Streamable
{

public:  
  
  // the actions to be scheduled in order to perform remote distribution of percepts and their integration
  enum ScheduleState   
  {
    idle,
    perceptDistributionReceiveAllowed,
    perceptDistributionAllowed,
    worldModelUpdateAllowed,
    sensorResettingAllowed,
    numberOfScheduleStates
  };

  enum Periodicity
  {
    // some tasks will only be triggered every n'th cycle
    executeEveryNthCycle = 4,
    periodicityIndex_sensorResetting = 0,
  };

  enum Timings
  {
    // one frame lasts ~66ms at 15Hz
    distributionStart        = 100,           // wait a little after start of the cycle so that all synchronized robots have the chance to receive    
    updateStart              = 300,           // time in ms after beginning of an update-cycle when the update on the global model has to be performed
    updateEnd                = 400,
    cycleLength              = 500,
    extendedCycleLength = cycleLength * executeEveryNthCycle,
    //maybe not needed:
  };


  ScheduleState scheduleState;
  long          timeUntilNextDistribution;
  long          timeUntilNextUpdate;


  CognitionScheduleData()
  {
    scheduleState = idle;
    timeUntilNextUpdate = 0;
    timeUntilNextDistribution = 0;
  }

  static const char* getStateName(ScheduleState state)
  {
    switch(state)
    {
    case idle: return "idle";
    case perceptDistributionReceiveAllowed: return "perceptDistributionReceiveAllowed";
    case perceptDistributionAllowed: return "perceptDistributionAllowed";
    case worldModelUpdateAllowed: return "worldModelUpdateAllowed";
    case sensorResettingAllowed: return "sensorResettingAllowed";
    default: return "unknown";
    }
  }
  /**
  * The method writes the schedule data onto the field view.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:CognitionScheduleData", "drawingOnField");
    DRAWTEXT("representation:CognitionScheduleData",-1000,-1500,10,ColorRGBA(255,255,255),"CognitionScheduleData: "<<getStateName(scheduleState));
  }

private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_ENUM(scheduleState,numberOfScheduleStates,CognitionScheduleData::getStateName);
    STREAM(timeUntilNextDistribution);
    STREAM(timeUntilNextUpdate);
    STREAM_REGISTER_FINISH();
  }
};


#endif //__CognitionScheduleData_h_
