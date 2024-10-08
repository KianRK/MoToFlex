/**
* @file RobotModel.h
*
* Declaration of class RobotModel
*
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander H�rtl</A>
*/

#ifndef RobotModel_H
#define RobotModel_H

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose3D.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#else
#include "math/Pose3D.h"
#include "bhumanstub.h"
#endif

/**
 * @class RobotModel
 *
 * Contains information about extremities.
 */
class RobotModel : public Streamable
{
  /** Streaming */
  virtual void serialize(In *in, Out *out)
  {
    STREAM_REGISTER_BEGIN();
      STREAM_ARRAY(limbs);
      STREAM(centerOfMass);
      STREAM(totalMass);
    STREAM_REGISTER_FINISH();
  }

public:
  enum Limb
  {
    head,
    upperArmLeft,
    lowerArmLeft,
    upperArmRight,
    lowerArmRight,
    upperLegLeft,
    lowerLegLeft,
    footLeft,
    upperLegRight,
    lowerLegRight,
    footRight,
    numOfLimbs
  };
  Pose3D limbs[numOfLimbs]; /**< Coordinate frame of the limbs of the robot relative to the robot's origin. */
  Vector3<double> centerOfMass; /**< Position of the center of mass (center of gravity) relative to the robot's origin. */
  double totalMass; /**< The mass of the robot. */

#ifndef WALKING_SIMULATOR
  /** Constructor */
  RobotModel() : totalMass(0) {}

  /** 
  * Constructs the RobotModel from given joint data. 
  * @param joints The joint data.
  * @param robotDimensions The dimensions of the robot.
  * @param massCalibration The mass calibration of the robot.
  */
  RobotModel(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /** 
  * Recalculates the RobotModel from given joint data. 
  * @param joints The joint data.
  * @param robotDimensions The dimensions of the robot.
  * @param massCalibration The mass calibration of the robot.
  */
  void setJointData(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

  /** Creates a 3-D drawing of the robot model. */
  void draw();
 #endif
};

#endif //RobotModel_H
