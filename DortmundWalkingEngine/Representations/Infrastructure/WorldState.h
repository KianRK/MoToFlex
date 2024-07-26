/**
* @file WorldState.h
*
* Definition of class WorldState
* 
* @author Gregor Jochmann
*/
#ifndef __WORLDSTATE_h_
#define __WORLDSTATE_h_

#include "Tools/Streams/Streamable.h"

#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"

class WorldState : public Streamable  {
public: 
  WorldState(){}; 
  ~WorldState(){};
  
  Pose2D playerPoses[8]; //0..3 - Red 1 to Red 4; 4..7 Blue 1 to Blue 4
  Vector2<double> playerSpeeds[8];
  GroundTruthBallModel ballModel;
  unsigned long timeStamp;

  //Getters (convenience access to public members, translate player+teamColor to array index)
  inline static int getArrayIndex(int player, int playerColor){
    return playerColor==TEAM_RED? player-1:player+3;
  };

  inline Vector2<double> getPlayerSpeed(int player,int playerColor) const {
    return playerSpeeds[getArrayIndex(player,playerColor)];
  };
	
  
  inline RobotPose getRobotPose(int player, int playerColor) const {    
    return playerPoses[getArrayIndex(player,playerColor)];
  };

  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM(timeStamp);
    STREAM(ballModel)
    STREAM(playerPoses[0])
    STREAM(playerSpeeds[0])
    STREAM(playerPoses[1])
    STREAM(playerSpeeds[1])
    STREAM(playerPoses[2])
    STREAM(playerSpeeds[2])
    STREAM(playerPoses[3])
    STREAM(playerSpeeds[3])
    STREAM(playerPoses[4])
    STREAM(playerSpeeds[4])
    STREAM(playerPoses[5])
    STREAM(playerSpeeds[5])
    STREAM(playerPoses[6])
    STREAM(playerSpeeds[6])
    STREAM(playerPoses[7])
    STREAM(playerSpeeds[7])
    STREAM_REGISTER_FINISH();
  }
};

#endif
