/**
 * @file RobotPoseHypotheses.h
 *
 * The file contains the definition of the class RobotPoseHypotheses.
 *
 * @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
 * @author <a href="mailto:Stefan.Czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
 */

#ifndef __RobotPoseHypotheses_h_
#define __RobotPoseHypotheses_h_

#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Matrix_mxn.h"
#include "Tools/Math/Matrix2x2.h"


/**
* @class RobotPoseHypothesis
* A single hypothesis about a robot position
*/
class RobotPoseHypothesis: public RobotPose
{
public:
  RobotPoseHypothesis(){};

  RobotPoseHypothesis(const Pose2D & pose, const Matrix_mxn<3,3> & covariance):
    covariance(covariance)
  {
    translation = pose.translation;
    rotation = pose.rotation;
  }

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
      STREAM_BASE(RobotPose);
      STREAM_ARRAY(covariance.content);
      STREAM_VECTOR(hypothesisOrigins);
    STREAM_REGISTER_FINISH();
  }

public:
  /** Covariance matrix about the position/orientation uncertainty*/
  Matrix_mxn<3,3> covariance;

  /** From which former hypotheses this one was generated (empty when newly generated from template) */
  std::vector<int> hypothesisOrigins;


  Matrix2x2<double> getPositionCovariance() const
  {
    return Matrix2x2<double>( covariance.getMember(0,0), covariance.getMember(0,1), 
                              covariance.getMember(1,0), covariance.getMember(1,1)  );
  }

  /** Assignment operator
  * @param other Another RobotPoseHypothesis
  * @return A reference to the object after the assignment
  */
  const RobotPoseHypothesis& operator=(const RobotPoseHypothesis& other)
  {
    (RobotPose&) *this = (const RobotPose&) other;
    covariance = other.covariance;
    hypothesisOrigins = other.hypothesisOrigins;
    return *this;
  }
};


/**
* @class RobotPoseHypotheses
* A set of hypotheses
*/
class RobotPoseHypotheses: public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
      STREAM_VECTOR(hypotheses);
    STREAM_REGISTER_FINISH();
  }

public:
  const int MAX_HYPOTHESES;                      /**< The maximum number of hypotheses */
  std::vector<RobotPoseHypothesis> hypotheses;   /**< The list of hypotheses */

  int indexOfBestHypothesis;
 
  /** Constructor*/
  RobotPoseHypotheses():MAX_HYPOTHESES(5),indexOfBestHypothesis(0) {}

  /** Assignment operator
  * @param other Another RobotPoseHypotheses
  * @return A reference to the object after the assignment
  */
  RobotPoseHypotheses& operator=(const RobotPoseHypotheses& other)
  {
    hypotheses = other.hypotheses;
    indexOfBestHypothesis = other.indexOfBestHypothesis;
    return *this;
  }

  const RobotPoseHypothesis& getBestHypothesis() const;

  /** Draws the hypotheses to the field view*/
  void draw();
};

#endif //__RobotPoseHypotheses_h_
