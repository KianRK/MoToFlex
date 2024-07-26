/**
* @file WMGDistributedPercepts.h
*
* This class contains obstacle/ball-percepts as gaussian. Used to distribute percepts
* amongst robots and to update the global model.
*
* TODO: performance: remove alloc, free from methods 
*
* @author <A href="mailto:c_rohde@web.de">Carsten Rohde</A>
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#ifndef __WMGDistributedPercepts_h_
#define __WMGDistributedPercepts_h_

#include "Modules/Modeling/WorldModelGenerator/tools/WMG_GaussianTools2D.h"

#include "Representations/Modeling/RobotPoseHypotheses.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/GaussianDistribution2D.h"

#include "Representations/Perception/ObstaclesPercept.h"
#include "Platform/GTAssert.h"

class WMGDistributedPercepts : public Streamable
{
public:
  enum Constants
  {
    maxNumOfDistributedObstacles = 8
  };


  struct Feature
  {
    Vector_n<double,2> location;
    Matrix_mxn<2,2> covariance;
    ObstaclesPercept::RobotType type;
    long ttl;
  };


  RobotPoseHypothesis  robotPose;
  bool                 relocalizedSinceLastUpdate;

  // In contrast to Carsten's thesis all features will 
  // be distributed in global coordinates since performance
  // will probably never allow to keep models for 
  // alternative robot pose hypotheses at the same time.
  Feature     obstacles[maxNumOfDistributedObstacles];
  int         numOfDistributedObstacles;

	Vector_n<double,4>  ball_location;
	Matrix_mxn<4,4>     ball_covariance;
  long                ball_ttl;
  bool                ballWasUpdated;


  unsigned int timeStampOfDistribution;
  
  WMGDistributedPercepts()
  {
    timeStampOfDistribution = 0;
    numOfDistributedObstacles = 0;
    relocalizedSinceLastUpdate = true;
    ballWasUpdated = false;
  }

  ~WMGDistributedPercepts()
  {
  }

  bool validate() const
  {
    for (int i=0;i<numOfDistributedObstacles;++i)
    {
      if (obstacles[i].location[0] == 0.0)
        return false;
    }
    if (robotPose.translation.abs() == 0)
      return false;

    return true;
  }

  // to be called after ball, obstacles and robotPose are filled in
  void transformAllFeaturesToGlobalCoords()
  {
    if (ballWasUpdated)
    {
      getTransformedBall_uncertain(tempLocation4D,tempCovar4D);
      ball_location = tempLocation4D;
      ball_covariance = tempCovar4D;
    }

    for (int i=0; i<numOfDistributedObstacles; i++)
    {
      getTransformedObstacle_uncertain(tempLocation2D,tempCovar2D,i);
      obstacles[i].location = tempLocation2D;
      obstacles[i].covariance = tempCovar2D;
    }
  }

private:

  Vector_n<double,2> tempLocation2D;
  Matrix_mxn<2,2>    tempCovar2D;
  Vector_n<double,4> tempLocation4D;
  Matrix_mxn<4,4>    tempCovar4D;  

  void getTransformedObstacle_certain(
    Vector_n<double,2> & transLocation, 
    Matrix_mxn<2,2> & transCovar, 
    int index, 
    const Pose2D& pose)
  {
    transformObstacleToGlobal_certain(transLocation, transCovar,obstacles[index].location, obstacles[index].covariance, pose);
  }

  void getTransformedObstacle_uncertain(
    Vector_n<double,2> & transLocation, 
    Matrix_mxn<2,2> & transCovar, 
    int obstacleIndex)
  {
    transformObstacleToGlobal_uncertain(transLocation,transCovar,
      obstacles[obstacleIndex].location, obstacles[obstacleIndex].covariance,
      robotPose, robotPose.covariance);
  }

  void getTransformedBall_certain(
    Vector_n<double,4> & transLocation,
    Matrix_mxn<4,4> & transCovar, 
    const Pose2D& pose)
  {
    transformBallToGlobal_certain(transLocation, transCovar,ball_location,ball_covariance,pose);
  }

  void getTransformedBall_uncertain(
    Vector_n<double,4> & transLocation,
    Matrix_mxn<4,4> & transCovar)
  {
    transformBallToGlobal_uncertain(transLocation,transCovar,
      ball_location, ball_covariance,
      robotPose, robotPose.covariance);
  }

  void getTransformedBall_uncertain(
    Vector_n<double,4> & transLocation,
    Matrix_mxn<4,4> & transCovar, 
    const Pose2D& pose,
    const Matrix_mxn<3,3> & poseCovar)
  {
    transformBallToGlobal_uncertain(transLocation,transCovar,
      ball_location, ball_covariance,
      pose, poseCovar);
  }

  void getTransformedObstacle_uncertain(
    Vector_n<double,2> & transLocation, 
    Matrix_mxn<2,2> & transCovar, 
    int obstacleIndex, 
    const Pose2D& pose, 
    const Matrix_mxn<3,3> & poseCovar)
  {
    transformObstacleToGlobal_uncertain(transLocation,transCovar,
      obstacles[obstacleIndex].location, obstacles[obstacleIndex].covariance,
      pose, poseCovar);
  }




  // keep in mind: particles track objects in global coordinates! 
  // own local percepts have to be transformed in each particles coordinates
  // other robots percepts just have to be transformed into global coordinates


  // obstacles are 2D!!
  void transformObstacleToGlobal_certain(
    Vector_n<double,2> & transLocation, 
    Matrix_mxn<2,2> & transCovar, 
    const Vector_n<double,2> & location, 
    const Matrix_mxn<2,2> & covar, 
    const Pose2D& pose)
  {
    double angle = pose.rotation;
    Matrix_mxn<2,2> rotation;    
    Matrix_mxn<2,2> temp2x2;  

    double c = cos(angle);
    double s = sin(angle);

    rotation.setMember(0,0,c);
    rotation.setMember(0,1,-s);
    rotation.setMember(1,0,s);    
    rotation.setMember(1,1,c);  

 
    // transform location
    //WMG_GSLTool::mul(transLocation, rotation, location);
    //*gsl_vector_ptr(transLocation,0) += pose.translation.x;
    //*gsl_vector_ptr(transLocation,1) += pose.translation.y;
    transLocation = rotation * location;
    transLocation[0] += pose.translation.x;
    transLocation[1] += pose.translation.y;

    // R*C*R^t
    //WMG_GSLTool::mul(temp2x2, rotation, covar);
    //gsl_matrix_transpose(rotation);
    //WMG_GSLTool::mul(transCovar, temp2x2, rotation);
    temp2x2 = rotation * covar;
    rotation = rotation.transpose();
    transCovar = temp2x2 * rotation;
  }

  //balls are 4D
  void transformBallToGlobal_certain(
    Vector_n<double,4> & transLocation,
    Matrix_mxn<4,4> & transCovar, 
    const Vector_n<double,4> & location, 
    const Matrix_mxn<4,4> & covar, 
    const Pose2D& pose)
  {  
    double angle = pose.rotation;
    Matrix_mxn<4,4> rotation;    
    Matrix_mxn<4,4> temp4x4;  

    // location and speed can be rotated inependently, thus 
    // R' = R 0
    //      0 R
    // where R is the 2D rotation matrix

    double c = cos(angle);
    double s = sin(angle);

    rotation.setMember(0,0,c);
    rotation.setMember(0,1,-s);
    rotation.setMember(1,0,s);    
    rotation.setMember(1,1,c);

    rotation.setMember(2,2,c);
    rotation.setMember(2,3,-s);
    rotation.setMember(3,2,s);    
    rotation.setMember(3,3,c);
    
     
    // transform location
    //WMG_GSLTool::mul(transLocation, rotation, location);
    //*gsl_vector_ptr(transLocation,0) += pose.translation.x;
    //*gsl_vector_ptr(transLocation,1) += pose.translation.y;
    transLocation = rotation * location;
    transLocation[0] += pose.translation.x;
    transLocation[1] += pose.translation.y;

    // transform covariance
    // R*C*R^t
    //WMG_GSLTool::mul(temp4x4, rotation, covar);
    //gsl_matrix_transpose(rotation);
    //WMG_GSLTool::mul(transCovar, temp4x4, rotation);
    temp4x4 = rotation * covar;
    rotation = rotation.transpose();
    transCovar = temp4x4 * rotation;
  }



  // transforms objectCovar and objectLocation given from a robotPose and covariance into global coordinates
  // used for other robots percepts
  void transformObstacleToGlobal_uncertain( 
    Vector_n<double,2> & transLocation, 
    Matrix_mxn<2,2> & transCovar, 
    const Vector_n<double,2> & location, 
    const Matrix_mxn<2,2> & covar, 
    const Pose2D& pose, 
    const Matrix_mxn<3,3> & poseCovar)
  {
    Matrix_mxn<2,2> rotation;
    Matrix_mxn<5,5> jacobian;
    Matrix_mxn<5,5> extdCovar;
    Matrix_mxn<5,5> extdCovarBuf;

    double mx    = location[0];
    double my    = location[1];

    double theta = pose.rotation;
    double cost  = cos(theta);
    double sint  = sin(theta);

    rotation.setMember(0,0,cost);
    rotation.setMember(0,1,-sint);
    rotation.setMember(1,0,sint);
    rotation.setMember(1,1,cost);

    jacobian.setToIdentity();

    //gsl_matrix* subjacobian = &gsl_matrix_submatrix(jacobian,0,0,2,2).matrix;
    //gsl_matrix_memcpy(subjacobian,rotation);
    jacobian.setMember(0,0,rotation.getMember(0,0));
    jacobian.setMember(0,1,rotation.getMember(0,1));
    jacobian.setMember(1,0,rotation.getMember(1,0));
    jacobian.setMember(1,1,rotation.getMember(1,1));

    //gsl_matrix_set(jacobian,0,2,1.0);
    //gsl_matrix_set(jacobian,1,3,1.0);
    jacobian.setMember(0,2,1.0);
    jacobian.setMember(1,3,1.0);

    //gsl_matrix_set(jacobian,0,4,-mx*sint - my*cost);
    //gsl_matrix_set(jacobian,1,4, mx*cost - my*sint);
    jacobian.setMember(0,4,-mx*sint - my*cost);
    jacobian.setMember(1,4, mx*cost - my*sint);

    //gsl_matrix* featureSubmatrix = &gsl_matrix_submatrix(extdCovar,0,0,2,2).matrix;
    //gsl_matrix_memcpy(featureSubmatrix,covar);
    for (int i=0; i<2; i++)
    {
      for (int j=0; j<2; j++)
      {
        extdCovar.setMember(i,j,covar.getMember(i,j));
      }
    }

    //gsl_matrix* poseSubmatrix    = &gsl_matrix_submatrix(extdCovar,2,2,3,3).matrix;
    //gsl_matrix_memcpy(poseSubmatrix,poseCovar);
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
      {
        extdCovar.setMember(2+i,2+j,poseCovar.getMember(i,j));
      }
    }

    // transform location
    //WMG_GSLTool::mul(transLocation, rotation, location);
    //*gsl_vector_ptr(transLocation,0) += pose.translation.x;
    //*gsl_vector_ptr(transLocation,1) += pose.translation.y;
    transLocation = rotation * location;
    transLocation[0] += pose.translation.x;
    transLocation[1] += pose.translation.y;

    //transform covar
    //WMG_GSLTool::mul(extdCovarBuf,jacobian,extdCovar);
    //gsl_matrix_transpose(jacobian);
    //WMG_GSLTool::mul(extdCovar,extdCovarBuf,jacobian);
    extdCovarBuf = jacobian * extdCovar;
    jacobian = jacobian.transpose();
    extdCovar = extdCovarBuf * jacobian;
    
    //gsl_matrix_memcpy(transCovar,featureSubmatrix);
    for (int i=0; i<2; i++)
    {
      for (int j=0; j<2; j++)
      {
        transCovar.setMember(i,j,extdCovar.getMember(i,j));
      }
    }


    //GaussianDistribution2D gd;
    //WMG_GSLTool::gsl2Framework_Matrix2x2(gd.covariance,covar);
    //WMG_GSLTool::gsl2Framework_Vector2(gd.mean,location);
    //WMG_GaussianTools2D::drawGaussian(gd,pose,ColorClasses::pink);

  }


  void transformBallToGlobal_uncertain(
    Vector_n<double,4> & transLocation, 
    Matrix_mxn<4,4> & transCovar, 
    const Vector_n<double,4> & location, 
    const Matrix_mxn<4,4> & covar, 
    const Pose2D& pose, 
    const Matrix_mxn<3,3> & poseCovar)
  {
    Matrix_mxn<2,2> rotation;
    Matrix_mxn<7,7> jacobian;
    Matrix_mxn<7,7> extdCovar;
    Matrix_mxn<7,7> extdCovarBuf;

    double mx    = location[0];
    double my    = location[1];
    double mvx   = location[2];
    double mvy   = location[3];


    double theta = pose.rotation;
    double cost  = cos(theta);
    double sint  = sin(theta);

    rotation.setMember(0,0,cost);
    rotation.setMember(0,1,-sint);
    rotation.setMember(1,0,sint);
    rotation.setMember(1,1,cost);

    jacobian.setToIdentity();

    //rotation matrix for location and speed
    //gsl_matrix* subjacobian = &gsl_matrix_submatrix(jacobian,0,0,2,2).matrix;
    //gsl_matrix_memcpy(subjacobian,rotation);
    jacobian.setMember(0,0,rotation.getMember(0,0));
    jacobian.setMember(0,1,rotation.getMember(0,1));
    jacobian.setMember(1,0,rotation.getMember(1,0));
    jacobian.setMember(1,1,rotation.getMember(1,1));
    //subjacobian = &gsl_matrix_submatrix(jacobian,2,2,2,2).matrix;
    //gsl_matrix_memcpy(subjacobian,rotation);
    jacobian.setMember(2,2,rotation.getMember(0,0));
    jacobian.setMember(2,3,rotation.getMember(0,1));
    jacobian.setMember(3,2,rotation.getMember(1,0));
    jacobian.setMember(3,3,rotation.getMember(1,1));

    //gsl_matrix_set(jacobian,0,4,1.0);
    //gsl_matrix_set(jacobian,1,5,1.0);
    jacobian.setMember(0,4,1.0);
    jacobian.setMember(1,5,1.0);

    //gsl_matrix_set(jacobian,0,6,-mx*sint - my*cost);
    //gsl_matrix_set(jacobian,1,6, mx*cost - my*sint);
    //gsl_matrix_set(jacobian,2,6,-mvx*sint - mvy*cost);
    //gsl_matrix_set(jacobian,3,6, mvx*cost - mvy*sint);
    jacobian.setMember(0,6,-mx*sint - my*cost);
    jacobian.setMember(1,6, mx*cost - my*sint);
    jacobian.setMember(2,6,-mvx*sint - mvy*cost);
    jacobian.setMember(3,6, mvx*cost - mvy*sint);


    //gsl_matrix* featureSubmatrix = &gsl_matrix_submatrix(extdCovar,0,0,4,4).matrix;
    //gsl_matrix_memcpy(featureSubmatrix,covar);
    for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
      {
        extdCovar.setMember(i,j,covar.getMember(i,j));
      }
    }

    //gsl_matrix* poseSubmatrix    = &gsl_matrix_submatrix(extdCovar,4,4,3,3).matrix;
    //gsl_matrix_memcpy(poseSubmatrix,poseCovar);
    for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
      {
        extdCovar.setMember(4+i,4+j,poseCovar.getMember(i,j));
      }
    }

    // transform location
    //gsl_matrix* rotation4D = &gsl_matrix_submatrix(jacobian,0,0,4,4).matrix;
    Matrix_mxn<4,4> rotation4D;
    for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
      {
        rotation4D.setMember(i,j,jacobian.getMember(i,j));
      }
    }

    //WMG_GSLTool::mul(transLocation, rotation4D, location);
    transLocation = rotation4D * location;

    //*gsl_vector_ptr(transLocation,0) += pose.translation.x;
    //*gsl_vector_ptr(transLocation,1) += pose.translation.y;
    transLocation[0] += pose.translation.x;
    transLocation[1] += pose.translation.y;
    

    //transform covar
    //WMG_GSLTool::mul(extdCovarBuf,jacobian,extdCovar);
    //gsl_matrix_transpose(jacobian);
    //WMG_GSLTool::mul(extdCovar,extdCovarBuf,jacobian);
    extdCovarBuf = jacobian * extdCovar;
    jacobian = jacobian.transpose();
    extdCovar = extdCovarBuf * jacobian;
    
    //gsl_matrix_memcpy(transCovar,featureSubmatrix);
    for (int i=0; i<4; i++)
    {
      for (int j=0; j<4; j++)
      {
        transCovar.setMember(i,j,extdCovar.getMember(i,j));
      }
    }



  }


public:

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();

    STREAM( robotPose ); 
    STREAM( relocalizedSinceLastUpdate);

    for (int i=0; i<maxNumOfDistributedObstacles; i++)
    {
      STREAM_ENUMASINT(obstacles[i].type);
      STREAM(obstacles[i].ttl);
      STREAM(obstacles[i].location);
      STREAM(obstacles[i].covariance);
    }
    STREAM( numOfDistributedObstacles);

    STREAM( ball_location);
    STREAM( ball_covariance);
    STREAM( ball_ttl);
    STREAM( ballWasUpdated);


    STREAM_REGISTER_FINISH();
  }


WMGDistributedPercepts& operator=(const WMGDistributedPercepts& other)
{
  if (this == &other)
    return *this;

  robotPose = other.robotPose;
  relocalizedSinceLastUpdate = other.relocalizedSinceLastUpdate;
  
  for (int i=0;i<other.numOfDistributedObstacles;++i)
  {
    obstacles[i].type = other.obstacles[i].type;
    obstacles[i].ttl  = other.obstacles[i].ttl;
    obstacles[i].location = other.obstacles[i].location;
    obstacles[i].covariance = other.obstacles[i].covariance;
  }
  numOfDistributedObstacles = other.numOfDistributedObstacles;

  ball_location = other.ball_location;
  ball_covariance = other.ball_covariance;

  ballWasUpdated = other.ballWasUpdated;
  ball_ttl       = other.ball_ttl;
  timeStampOfDistribution = other.timeStampOfDistribution;

  return *this;
}

};
#endif
