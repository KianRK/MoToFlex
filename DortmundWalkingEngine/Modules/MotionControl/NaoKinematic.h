
/** 
* @file Modules/MotionControl/NaoKinematic.h
* This file implements the inverse kinematic.
* @author <A href="mailto:Stefan.Czarnetzki@uni-dortmund.de">Stefan Czarnetzki</A>
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#ifndef __NaoKinematic_h_
#define __NaoKinematic_h_

#include <fstream>

using namespace std;

#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Common.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Vector3.h"
#include "Tools/RingBufferWithSum.h"


MODULE(NaoKinematic)
  REQUIRES(RobotDimensions)
  REQUIRES(JointData)
  REQUIRES(KinematicRequest)
  PROVIDES_WITH_MODIFY(KinematicOutput)
END_MODULE

class CombinedIterativeNaoKinematic;

class RCXPKinematicInterface : public Streamable
{
public: JointRequest jointRequest;
		double kinematicRequest[12];
		bool calculated;

  RCXPKinematicInterface()
  {
	  calculated = false;

	  for (int i=0;i<12;i++)
	  {
		  kinematicRequest[i] = 0;
	  }
  }

  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN();
    STREAM(jointRequest);
	STREAM_ARRAY(kinematicRequest);
	STREAM(calculated);
    STREAM_REGISTER_FINISH();
  }
};

#else
#include "math/Vector3.h"
#include "bhumanstub.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"

/**
 * @class Robot
 * Base class for NaoKinematic. This is just a stub to be compatible to
 * BHuman framework, where the data members are defined in the blackboard.
 */
class NaoKinematicBase
{
public:

	/** The desired foot positions. */
	KinematicRequest	theKinematicRequest;

	/** Actual angles. */
	JointData			theJointData;

	/** Dimensions of the robot. */
	RobotDimensions		theRobotDimensions;
};


#endif

/**
 * @class Robot
 * The inverse kinematic for the Nao robot.
 */
class NaoKinematic : public NaoKinematicBase
{
public:
  friend class CombinedIterativeNaoKinematic;

  /** Constructor */
  NaoKinematic();
  /** Desctructor */
  ~NaoKinematic();

  /**
   * Calculates the angles from the given foot positions in theKinematicRequest.
   * \param walkingEngineOutput Filled with the angles.
   */
   void update(KinematicOutput& walkingEngineOutput);
#ifndef WALKING_SIMULATOR
   void update(RawKinematicOutput& rawKinematicOutput);
	
   RCXPKinematicInterface rcxpKinematic;
#endif
private:
	
	Vector3<double> checkConstraints(Vector3<double> lf, double lfr, Vector3<double> rf, double rfr, bool left);
	/**
	 * Calculates the angles the first leg. It depends on the request 
	 * which leg is the first leg. E.g. in single support phase the standing
	 * leg is the first leg.
	 * \param whichSideJoint0 Index of first joint of leg.
	 * \param position The desired foot position.
	 * \param rotation The desired foot rotation.
	 * \param jointRequest Filled with the calculated angles.
	 */
  void calcLegJoints(JointData::Joint whichSideJoint0, const Vector3<double>& position, const Vector3<double>& rotation, JointRequest& jointRequest);

	/** This method implemenets a mixed kinematic. Here you can set t0, given by the calcLegJoints, to be
	 * compatible to the real Joint 0 (where the angle of both legs has to be equal)
	 * This resuts in a z rotation of the foot.
	 * \param whichSideJoint0 Index of first joint of leg.
	 * \param position The desired foot position.
	 * \param rotation The desired foot rotation. z is ignored here.
	 * \param jointRequest Filled with the calculated angles.
	 */
		void calcLegJoints(JointData::Joint whichSideJoint0, const Vector3<double>& position, const Vector3<double>& rotation, double t0, JointRequest& jointRequest);
		ofstream logfile;	  
};

#endif 
