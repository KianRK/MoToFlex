/**
* @file Representations/MotionControl/MotionRequest.h
* This file declares a class that represents the motions that can be requested from the robot.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#ifndef __MotionRequest_H__
#define __MotionRequest_H__

#include "Modules/MotionControl/DortmundWalkingEngine/WalkingInformations.h"


#ifndef WALKING_SIMULATOR
#include "SpecialActionRequest.h"
#include "WalkRequest.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugSymbolsLog.h"
#else
#include "bhumanstub.h"
#endif

const double walkMaxBackwardSpeed = 30,
             walkMaxForwardSpeed = 100,
             walkMaxLeftRightSpeed = 40,
             walkMaxRotationSpeed = 0.6,
			 walkMaxPitchAngle = 0.2;

/**
* @class MotionRequest
* A class that represents the motions that can be requested from the robot.
*/
class MotionRequest : public Streamable
{
protected:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM_ENUM(motion, numOfMotions, MotionRequest::getMotionName);
	STREAM_ENUM(standType, numOfStandTypes, MotionRequest::getStandTypeName);
    STREAM(specialActionRequest);
	STREAM(walkRequest);
	STREAM(kickDirection);
	STREAM(kickTime);
    STREAM_REGISTER_FINISH();
  }

public:
  enum Motion
  {    
    specialAction,    
	walk,
    numOfMotions
  };

  Motion motion; /**< The selected motion. */
  SpecialActionRequest specialActionRequest; /**< The special action request, if it is the selected motion. */
  WalkRequest walkRequest; /**< The walk request, if it is the selected motion. */
  StandType standType; /**< How should the robot stand when speed is 0? */
  double kickDirection; /**< Kick direction */
  double kickTime; /**< Time to execute the kick (the shorter the faster) */

  /** 
  * The function returns names of motions.
  * @param motion The motion the name of which is returned.
  * @return The corresponding name.
  */
  static const char* getMotionName(Motion motion)
  {
    switch(motion)
    {    
    case specialAction: return "specialAction";    
	case walk : return "walk";
    default: return "unknown";
    }
  }

  /** 
  * The function returns names of standType.
  * @param motion The standType the name of which is returned.
  * @return The corresponding name.
  */
  static const char* getStandTypeName(StandType st)
  {
    switch(st)
    {
		case doubleSupport: return "doubleSupport";
		case leftSingleSupport: return "leftSingleSupport";
		case rightSingleSupport : return "rightSingleSupport";
		default: return "unknown";
    }
  }

  /** 
  * Default constructor.
  */
  MotionRequest() : motion(specialAction) {}

#ifndef WALKING_SIMULATOR
/**
* Prints the motion request to a readable string. (E.g. "walk: 100mm/s 0mm/s 0°/s")
* @param destination The string to fill
*/
void debugLog()
{
	{
	std::string result="";
	result += getMotionName(motion);
	char tmpString[100]="";

	  switch (motion)
	  {
			case walk:
				sprintf(tmpString,":%.0lfmm/s%.0lfmm/s%.0lf°/s",
					walkRequest.speed.translation.x, walkRequest.speed.translation.y,
					toDegrees(walkRequest.speed.rotation));
				break;
			case specialAction:
				sprintf(tmpString,":%.100s", 
					SpecialActionRequest::getSpecialActionName(specialActionRequest.specialAction));
				break;
			case numOfMotions: // this enum can be ignored
				break;
		}
		result += tmpString;
		DebugSymbolsLog::print(result);
		}
}


  /**
  * Prints the motion request to a readable string. (E.g. "walk: 100mm/s 0mm/s 0°/s")
  * @param destination The string to fill
  */
  void printOut(char* destination) const
  {
    strcpy(destination, getMotionName(motion));
    destination += strlen(destination);
    switch (motion)
    {    
    case specialAction:
      sprintf(destination, ": %s", SpecialActionRequest::getSpecialActionName(specialActionRequest.specialAction));
      break;
    case walk:
      sprintf(destination, ": %.0lfmm/s %.0lfmm/s %.0lf°/s %s",
        walkRequest.speed.translation.x, walkRequest.speed.translation.y,
	      toDegrees(walkRequest.speed.rotation),walkRequest.forceOmniOnly ? "(forceOmni)" : "");
    break;
    default:
      break;
    }
  }

  /** Draws something*/
  void draw() 
  {
    DECLARE_DEBUG_DRAWING("representation:MotionRequest", "drawingOnField"); // drawing of a request walk vector
	if(motion == walk)
    {
      Vector2<double> translation = walkRequest.speed.translation;
      translation *= 10.0;

      ARROW("representation:MotionRequest", 0, 0, 
        translation.x, translation.y, 4, Drawings::ps_solid, ColorRGBA(0xcd, 0, 0));
      if(walkRequest.speed.rotation != 0.0)
      {
        translation.y = 0;
        translation.x = 750;
        translation.rotate(walkRequest.speed.rotation);
        ARROW("representation:MotionRequest", 0, 0, 
          translation.x, translation.y, 4, Drawings::ps_solid, ColorRGBA(0xcd, 0, 0, 127));
      }
    }
  }
  #endif
};

#endif // __MotionRequest_H__
