/*
* position and type of line crossings on the field
*
* @author <A href=mailto:stefan.czarnetzki@tu-dortmund.de>Stefan Czarnetzki</A>
*
*/


#ifndef NDLineCrossingsTable_h
#define NDLineCrossingsTable_h


#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Pose2D.h"
#include "Representations/Perception/LineCrossingsPercept.h"
#include "Representations/Modeling/SampleTemplates.h"
#include <list>

class NDLineCrossingsTable
{
public:
  enum CrossingClass{
    virtualCrossing,
    lCrossing,
    tCrossing,
    xCrossing,
    numOfCrossingClasses
  };

  NDLineCrossingsTable();


  void initialize(const FieldDimensions& field);

  bool getTemplatesFromSingleCrossing(const LineCrossingsPercept::LineCrossing& crossingPoint,
                                      std::vector<SampleTemplate> & templates) const;

  
  bool getClosestPoint(   const Pose2D& crossingPoseInAbsoluteCoords,
                          const LineCrossingsPercept::LineCrossing& crossingPoint,
                          const double & distanceTrust,
                          const double & orientationTrust,
                          Pose2D & resultingCrossingPose,
                          CrossingClass & resultingCrossingClass,
                          double & resultingSimilarityExponent,
                          double & resultingSimilarityExponentWithoutClassification) const;
  
  void drawCrossingsForTemplates() const;
  void drawCrossingMapping() const;
  void testMapping() const;

  std::list<double> getPossibleOriOffsets(const LineCrossingsPercept::LineCrossing& crossing, CrossingClass& crossingClass);

  // for templates from crossings
  enum{
    maxNumOfCrossings = 10 // should be 4, shouldn't it? -> V-crossings (the rest is never more than 2 when considering orientation)
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



  // for crossing mapping (for self loc.)
  struct CrossingTemplate
  {
    CrossingClass crossingClass;
    Vector2<double> positionOnField;
    // double angleOnField = 0;
    LineCrossingsPercept::CrossingCharacteristic side[4]; // sides in mathematical positive order beginning from the one directly at angleOnField
  };

  int numOfCrossingsInArea[10];
  CrossingTemplate crossingsInArea[10][4];
  
  int xAreaThreshold; // decide distance between middle line crossings and penalty&groundline crossings
  int yAreaThreshold; // (between area 0 and 1) (decide distance between crossings on left ground line those on right penalty line)

  // should be classificationPenaltyTable[3][2], but for easier lookup use [3][3]
  double classificationPenaltyTable[3][3]; // seen vs. real (should be 0 for correct match, dontKnows and line2noLine must be specified)

  void fillCrossingTemplate(CrossingTemplate & t, const CrossingClass& crossingClass, const double& x, const double& y,
                            int s0, 
                            int s1,
                            int s2,
                            int s3)
  {
    fillCrossingTemplate(t,crossingClass,x,y,
      s0 ? LineCrossingsPercept::lineOnThisSide : LineCrossingsPercept::noLineOnThisSide,
      s1 ? LineCrossingsPercept::lineOnThisSide : LineCrossingsPercept::noLineOnThisSide,
      s2 ? LineCrossingsPercept::lineOnThisSide : LineCrossingsPercept::noLineOnThisSide,
      s3 ? LineCrossingsPercept::lineOnThisSide : LineCrossingsPercept::noLineOnThisSide);
  }

  void fillCrossingTemplate(CrossingTemplate & t, const CrossingClass& crossingClass, const double& x, const double& y,
                            LineCrossingsPercept::CrossingCharacteristic s0, 
                            LineCrossingsPercept::CrossingCharacteristic s1,
                            LineCrossingsPercept::CrossingCharacteristic s2,
                            LineCrossingsPercept::CrossingCharacteristic s3)
  {
    t.positionOnField.x = x;
    t.positionOnField.y = y;
    t.side[0] = s0;
    t.side[1] = s1;
    t.side[2] = s2;
    t.side[3] = s3;
    t.crossingClass = crossingClass;
  }



  /*
  The idea: 
  Find the candidate maximizing the likelyhood.
  Divide the field into sub-areas with the crossings that are relevant for the decision.
  Examples of crossings that are NOT relevant: 
  - T-crossings at penalty area; both have the same characteristics 
    and the same orientation, so distance (->area) is sufficient for decision
    -> only one of them in area 2, the other one in area 3, but both L's in both 2 and 3
  - crossings that are simply too far away. Mapping something to the
    other half of the field because the characteristics fit a bit better
    is considered to be unnecessary.

  The areas should provide a relevant subset of closest crossings from which the ML candidate is found.

  L        T    |   |   |    T        L
                |   |   |
       0        | 1 | 2 |        3
  V        L    |   |   |    L        V
                |   |   |
-----------------------------------------
                    |
                    |
  T             X   |   X             T
       4            |            5
                    |
-----------------------------------------
                |   |   |
  V        L    |   |   |    L        V
       6        | 7 | 8 |        9
                |   |   |
  L        T    |   |   |    T        L


  */

  void initMappingAreas(const FieldDimensions& fieldDimensions);

};
#endif
