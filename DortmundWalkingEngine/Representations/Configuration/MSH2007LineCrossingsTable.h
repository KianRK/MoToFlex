/*
* position and type of line crossings on the field
*
* @author <A href=mailto:c_rohde@web.de>Carsten Rohde</A>
* @author <A href=mailto:judith-winter@web.de>Judith Winter</A>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
* @author Max Risler
*/


#ifndef MSH2007LineCrossingsTable_h
#define MSH2007LineCrossingsTable_h


#include  "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Pose2D.h"
#include "Representations/Perception/PointsPercept.h"
#include "Representations/Perception/LineCrossingsPercept.h"

#include <list>

class MSH2007LineCrossingsTable
{
public:
  enum CrossingClass{
    virtualCrossing,
    lCrossing,
    tCrossing,
    xCrossing,
    outerLCrossing,
    outerTCrossing, //unused, they dont exist on the field
    outerVirtualCrossing,
    falseCrossing,
    numOfCrossingClasses
  };

  MSH2007LineCrossingsTable();

  /**
  * FieldDimensions is not necessarily provided during construction, since this class has
  * not yet been properly embedded into the nao-framework
  * don't forget to initialize
  */
  void initialize(const FieldDimensions& field);

  Vector2<double> getClosestPoint(const Vector2<double>& point, CrossingClass crossingClass, int orientation) const;
  //Vector2<double> getClosestPoint(const Pose2D& pose, const LinesPercept::LineCrossing& crossingPoint, CrossingClass& crossingClass) const;
  Pose2D getClosestPoint(const Pose2D& pose, double& maxAngleDelta, /*const LinesPercept::LineCrossing& crossingPoint*/ const LineCrossingsPercept::LineCrossing& crossingPoint , CrossingClass& crossingClass) const;
  
  std::list<double> getPossibleOriOffsets(const LineCrossingsPercept::LineCrossing& crossing, CrossingClass& crossingClass);

  void draw() const;

  static int NO_POINT_DISTANCE;

  enum{
    maxNumOfCrossings = 10
  };

  int numOfCrossings[numOfCrossingClasses][4];
  Vector2<double> crossings[numOfCrossingClasses][4][maxNumOfCrossings];

private:

  // private methods
  void addCrossing(CrossingClass crossingClass, int orientation, double x, double y)
  {
    int& num = numOfCrossings[crossingClass][orientation];
    crossings[crossingClass][orientation][num].x = x;
    crossings[crossingClass][orientation][num].y = y;
    num++;
  }

  void initVirtualCrossings(const FieldDimensions& fieldDimensions);
  void initTCrossings(const FieldDimensions& fieldDimensions);
  void initLCrossings(const FieldDimensions& fieldDimensions);
  void initXCrossings(const FieldDimensions& fieldDimensions);
  void initFalseCrossings(const FieldDimensions& fieldDimensions);

  //void updateClosestPoint(Vector2<double>& minimum, double& minDist, const Vector2<double>& pose, CrossingClass crossingClass, int orientation, CrossingClass& minCrossingClass) const;
  void updateClosestPoint(Pose2D& result, double& maxAngleDelta, bool characterized, double& minDist, const Vector2<double>& pose, CrossingClass crossingClass, int orientation, CrossingClass& minCrossingClass) const;
};
#endif
