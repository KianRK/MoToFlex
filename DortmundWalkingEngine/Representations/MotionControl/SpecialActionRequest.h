/**
* @file Representations/MotionControl/SpecialActionRequest.h
* This file declares a class to represent special action requests.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#ifndef __SpecialActionRequest_H__
#define __SpecialActionRequest_H__

#include "Tools/Streams/Streamable.h"
#include "Tools/Settings.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/RobotName.h"

/**
* @class SpecialActionRequest
* The class represents special action requests.
*/
class SpecialActionRequest : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM_ENUM(specialAction, numOfSpecialActions, getSpecialActionName);
    STREAM(mirror);
    STREAM_REGISTER_FINISH();
  }

public:
  /** ids for all special actions */
  enum SpecialActionID
  {
    activeStand,
    bow,
    cheering,
    demoHardness,
    demoJesus,
    demoUpright,
	falling,
    footsensortest,
	goalkeeperDefend,
	haka,
//kicks
  kickInner,
  kickMiddle,
  kickOuter,
	kick_0Hard,
	kick_1Hard,
	kick_2Hard,
	kick_3Hard,
	motionDemo1,
	motionDemo2,
	motionDemo3,
	motionDemo4,
	motionDemo5,
	motionDemo6,
	no,
	pitch,
	playDead,
  playDeadKeeper,
	playDeadR,
	shakehand,
	sideKickHard,
	sidePass,
	sitBackAndWait,
	stand,
	standStraight,
	standUpBackNao,
	standUpFrontNao,
	wave_left,
	wave_right,
	yes,

    numOfSpecialActions
  };

  SpecialActionID specialAction; /**< The special action selected. */
  bool mirror; /**< Mirror left and right. */

  /**
  * Default constructor.
  */
  SpecialActionRequest() : specialAction(playDead), mirror(false) {}

  void specialize(RobotInfo::RobotModel model, RobotName robotName)
  {
/*#ifdef TARGET_SIM
    if(specialAction == kickBremenLeftNao)
      specialAction = kickBremenNaoSimulator;

#endif*/
  }

  /** 
  * The function returns names for special action ids.
  * @param id The special action id the name of which is returned.
  * @return The corresponding name.
  */
  static const char* getSpecialActionName(SpecialActionID id)
  {
    switch (id)
    {
	case activeStand: return "activeStand";
	case bow: return "bow";
	case cheering: return "cheering";
	case demoHardness: return "demoHardness";
	case demoJesus: return "demoJesus";
	case demoUpright: return "demoUpright";
	case falling: return "falling";
	case footsensortest: return "footsensortest";
	case goalkeeperDefend: return "goalkeeperDefend";
	case haka: return "haka";
  case kickInner: return "kickInner";
  case kickMiddle: return "kickMiddle";
  case kickOuter: return "kickOuter";
	case kick_0Hard: return "kick_0Hard";
	case kick_1Hard: return "kick_1Hard";
	case kick_2Hard: return "kick_2Hard";
	case kick_3Hard: return "kick_3Hard";
	case motionDemo1: return "motionDemo1";
	case motionDemo2: return "motionDemo2";
	case motionDemo3: return "motionDemo3";
	case motionDemo4: return "motionDemo4";
	case motionDemo5: return "motionDemo5";
	case motionDemo6: return "motionDemo6";
	case no: return "no";
	case pitch:return "pitch";
	case playDead: return "playDead";
  case playDeadKeeper: return "playDeadKeeper";
	case playDeadR: return "playDeadR";
	case shakehand: return "shakehand";
	case sideKickHard : return "sideKickHard";
	case sidePass : return "sidePass";
	case sitBackAndWait: return "sitBackAndWait";
	case stand: return "stand";
	case standStraight: return "standStraight";
	case standUpBackNao: return "standUpBackNao";
	case standUpFrontNao: return "standUpFrontNao";
	case wave_left: return "wave_left";
	case wave_right: return "wave_right";
	case yes: return "yes";

    default: return "unknown";
    }
  }

  /**
  * The function searches the id for a special action name.
  * @param name The name of the special action.
  * @return The corresponding id if found, or numOfSpecialActions if not found.
  */
  static SpecialActionID getSpecialActionFromName(const char* name)
  {
    for(int i = 0; i < numOfSpecialActions; ++i)
      if(!strcmp(name, getSpecialActionName(SpecialActionID(i))))
        return SpecialActionID(i);
    return numOfSpecialActions;
  }
};

#endif // __SpecialActionRequest_H__
