#ifndef __KinematicRequest_H__
#define __KinematicRequest_H__


#include "Representations/Infrastructure/JointData.h"

#ifdef WALKING_SIMULATOR
#include "bhumanstub.h"
#endif

//#define USEARMS

/**
* @class KinematicRequest
* A class that represents a kinematic request.
*/
class KinematicRequest : public Streamable
{
private:
  virtual void serialize( In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN( );
    STREAM_ENUM(kinematicType, numOfKinematicType, KinematicRequest::getKinematicTypeName);
    STREAM(offsets);
    STREAM_ARRAY(body);
    STREAM_ARRAY(leftFoot);
    STREAM_ARRAY(rightFoot);
    STREAM_REGISTER_FINISH();
  }

public:
  enum KinematicType {
      feet,              // Position of the feet relative to the body.
	  bodyAndLeftFoot,    // Position of the body and left foot relative to the right foot.
	  bodyAndRightFoot,   // Position of the body and right foot relative to the left foot.
      numOfKinematicType
  };
  
  /** 
  * The function returns names of kinematic types.
  * @param motion The kinematic type the name of which is returned.
  * @return The corresponding name.
  */
  static const char* getKinematicTypeName(KinematicType type)
  {
    switch(type)
    {
    case feet: return "feet";
    case bodyAndLeftFoot: return "bodyAndLeftFoot";
    case bodyAndRightFoot : return "bodyAndRightFoot";
    default: return "unknown";
    }
  }

  enum armEnum{
	  left0,
	  left1,
	  left2,
	  left3,
	  right0,
	  right1,
	  right2,
	  right3,
	  numOfArmAngles
  };
	  
  KinematicRequest(){  
    int i;
    for(i = 0; i < offsets.numOfJoints; ++i) offsets.angles[i] = 0;
    for (i = 0; i < 6; ++i) {
      body[i] = 0;
      leftFoot[i] = 0;
      rightFoot[i] = 0;
    }
  }

  /** kinematic type */
  KinematicType kinematicType;

  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  float body[6]; 
  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  float leftFoot[6];
  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  float rightFoot[6];

  JointData offsets;
};

#endif // __WalkRequest_H__
