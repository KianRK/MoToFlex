/*
* position and type of line crossings on the field
*
* @author <A href=mailto:stefan.czarnetzki@tu-dortmund.de>Stefan Czarnetzki</A>
*
*/


#include "NDLineCrossingsTable.h"

#include "Tools/Debugging/Modify.h"


NDLineCrossingsTable::NDLineCrossingsTable()
{
  for(int i = 0; i < numOfCrossingClasses; i++)
    for(int j = 0; j < 4; j++)
      numOfCrossings[i][j] = 0;
  for (int i = 0; i < 10; i++)
    numOfCrossingsInArea[i] = 0;
}

void NDLineCrossingsTable::initialize(const FieldDimensions& field)
{
  initVirtualCrossings(field);
  initTCrossings(field);
  initLCrossings(field);
  initXCrossings(field); 

  initMappingAreas(field);

  classificationPenaltyTable[LineCrossingsPercept::noLineOnThisSide][LineCrossingsPercept::noLineOnThisSide] = 0;
  classificationPenaltyTable[LineCrossingsPercept::dontKnow][LineCrossingsPercept::noLineOnThisSide] = 1;
  classificationPenaltyTable[LineCrossingsPercept::lineOnThisSide][LineCrossingsPercept::noLineOnThisSide] = 2;

  classificationPenaltyTable[LineCrossingsPercept::noLineOnThisSide][LineCrossingsPercept::lineOnThisSide] = 2;
  classificationPenaltyTable[LineCrossingsPercept::dontKnow][LineCrossingsPercept::lineOnThisSide] = 1;
  classificationPenaltyTable[LineCrossingsPercept::lineOnThisSide][LineCrossingsPercept::lineOnThisSide] = 0;

  // are never needed! (only there for easier lookup)
  classificationPenaltyTable[LineCrossingsPercept::noLineOnThisSide][LineCrossingsPercept::dontKnow] = 100;
  classificationPenaltyTable[LineCrossingsPercept::dontKnow][LineCrossingsPercept::dontKnow] = 100;
  classificationPenaltyTable[LineCrossingsPercept::lineOnThisSide][LineCrossingsPercept::dontKnow] = 100;
  //
}

void NDLineCrossingsTable::drawCrossingsForTemplates() const
{
  DECLARE_DEBUG_DRAWING("NDLineCrossingsTable:drawCrossingsForTemplates", "drawingOnField");
  for(int i = 0; i < numOfCrossingClasses; i++)
    for(int j = 0; j < 4; j++)
      for (int n=0; n<numOfCrossings[i][j]; n++)
      {
        int side1, side2, side3, side4;
        switch(i)
        {
        case virtualCrossing:
          side1 = LineCrossingsPercept::lineOnThisSide;
          side2 = LineCrossingsPercept::noLineOnThisSide;
          side3 = LineCrossingsPercept::lineOnThisSide;
          side4 = LineCrossingsPercept::noLineOnThisSide;
          break;
        case lCrossing:
          side1 = LineCrossingsPercept::lineOnThisSide;
          side2 = LineCrossingsPercept::lineOnThisSide;
          side3 = LineCrossingsPercept::noLineOnThisSide;
          side4 = LineCrossingsPercept::noLineOnThisSide;
          break;
        case tCrossing:
          side1 = LineCrossingsPercept::lineOnThisSide;
          side2 = LineCrossingsPercept::lineOnThisSide;
          side3 = LineCrossingsPercept::lineOnThisSide;
          side4 = LineCrossingsPercept::noLineOnThisSide;
          break;
        case xCrossing:
        default:
          side1 = LineCrossingsPercept::lineOnThisSide;
          side2 = LineCrossingsPercept::lineOnThisSide;
          side3 = LineCrossingsPercept::lineOnThisSide;
          side4 = LineCrossingsPercept::lineOnThisSide;
          break;
        };

        COMPLEX_DRAWING("NDLineCrossingsTable:drawCrossingsForTemplates",
        {
          Vector2<double> intersectionOnFieldGlobal = crossings[i][j][n];
          double angleOnFieldGlobal = j*pi_2;

          CIRCLE("NDLineCrossingsTable:drawCrossingsForTemplates",intersectionOnFieldGlobal.x,intersectionOnFieldGlobal.y,60,6,Drawings::ps_solid,ColorClasses::red,Drawings::bs_null,ColorClasses::red);


          // draw characteristics of the sides: white->lineOnThisSide, black->noLineOnThisSide, light_gray->dontKnow
          LINE("NDLineCrossingsTable:drawCrossingsForTemplates", // side1
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side1]
          );
          LINE("NDLineCrossingsTable:drawCrossingsForTemplates",// side2
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal+pi_2)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal+pi_2)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side2]
          );
          LINE("NDLineCrossingsTable:drawCrossingsForTemplates",// side3
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal+pi)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal+pi)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side3]
          );
          LINE("NDLineCrossingsTable:drawCrossingsForTemplates",// side4
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal+pi3_2)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal+pi3_2)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side4]
          );    
        }
        );// end complex drawing

      }

}


void NDLineCrossingsTable::drawCrossingMapping() const
{
  DECLARE_DEBUG_DRAWING("NDLineCrossingsTable:drawCrossingMapping", "drawingOnField");
  int whichArea = 0;
  MODIFY("NDLineCrossingsTable:CrossingMapping:whichAreaToDraw", whichArea);

  if (whichArea < 0 || whichArea >= 10)
  {
    return;
  }

  // this could also go into the COMPLEX_DRAWING, but for some reason does not compile then...
  Vector2<int> upperLeft(3700,2700),lowerRight(-3700,-2700);
  switch(whichArea)
  {
    case 0:
    case 1:
    case 2:
    case 3:
      lowerRight.x =  xAreaThreshold;
      break;
    case 4:
    case 5:
      upperLeft.x  =  xAreaThreshold;
      lowerRight.x = -xAreaThreshold;
      break;
    case 6:
    case 7:
    case 8:
    case 9:
      upperLeft.x  = -xAreaThreshold;
      break;
    default:
      break;
  }
  switch(whichArea)
  {
    case 0:
    case 6:
      lowerRight.y =  yAreaThreshold;
      break;
    case 3:
    case 9:
      upperLeft.y =  -yAreaThreshold;
      break;
    case 1:
    case 7:
      upperLeft.y =  yAreaThreshold;
    case 4:
      lowerRight.y =  0;
      break;
    case 2:
    case 8:
      lowerRight.y = -yAreaThreshold;
    case 5:
      upperLeft.y  =  0;
      break;
    default:
      break;
  }

  COMPLEX_DRAWING("NDLineCrossingsTable:drawCrossingMapping",
  {
    RECTANGLE("NDLineCrossingsTable:drawCrossingMapping",upperLeft.x,upperLeft.y,lowerRight.x,lowerRight.y,50,Drawings::ps_solid, ColorRGBA(128,0,255));

    for(int i = 0; i < numOfCrossingsInArea[whichArea]; i++)
    {
      const CrossingTemplate & c = crossingsInArea[whichArea][i];
      double x = crossingsInArea[whichArea][i].positionOnField.x;
      double y = crossingsInArea[whichArea][i].positionOnField.y;

      CIRCLE( "NDLineCrossingsTable:drawCrossingMapping",
              x,
              y,
              60,40,Drawings::ps_solid,ColorClasses::red,Drawings::bs_null,ColorClasses::red);

      int fontsize = 30;
      DRAWTEXT( "NDLineCrossingsTable:drawCrossingMapping", x + 2*fontsize, y + 2*fontsize, fontsize, ColorRGBA(0,0,0), i );


      // draw characteristics of the sides: white->lineOnThisSide, black->noLineOnThisSide, light_gray->dontKnow
      LINE("NDLineCrossingsTable:drawCrossingMapping", // side0
        x,
        y,
        x + 200,
        y,
        40,
        Drawings::ps_solid,
        c.side[0] ? ColorRGBA(255,0,255) : ColorRGBA(0,0,0)
      );
      LINE("NDLineCrossingsTable:drawCrossingMapping",// side1
        x,
        y,
        x,
        y + 200,
        40,
        Drawings::ps_solid,
        c.side[1] ? ColorRGBA(255,0,255) : ColorRGBA(0,0,0)
      );
      LINE("NDLineCrossingsTable:drawCrossingMapping",// side2
        x,
        y,
        x - 200,
        y,
        40,
        Drawings::ps_solid,
        c.side[2] ? ColorRGBA(255,0,255) : ColorRGBA(0,0,0)
      );
      LINE("NDLineCrossingsTable:drawCrossingMapping",// side3
        x,
        y,
        x,
        y - 200,
        40,
        Drawings::ps_solid,
        c.side[3] ? ColorRGBA(255,0,255) : ColorRGBA(0,0,0)
      );
    }
  }
  ); // end of complex drawing

  testMapping();
}

void NDLineCrossingsTable::testMapping() const
{  
  DECLARE_DEBUG_DRAWING("NDLineCrossingsTable:drawTestCrossingMapping", "drawingOnField");
  LineCrossingsPercept::LineCrossing point;
  point.side[0] = LineCrossingsPercept::dontKnow;
  point.side[1] = LineCrossingsPercept::lineOnThisSide;
  point.side[2] = LineCrossingsPercept::noLineOnThisSide;
  point.side[3] = LineCrossingsPercept::lineOnThisSide;
  point.locationOnField.x = 2500;
  point.locationOnField.y = 1000;
  point.angleOnField = 0;
  point.angleInImage1 = 0;
  point.angleInImage2 = 0;
  //point.borderLine = false;
  //point.fieldLine = true;
  point.outOfImage = false;
  MODIFY("NDLineCrossingsTable:CrossingMapping:testCrossingForMapping", point);
  Pose2D testPose(point.angleOnField,Vector2<double>(point.locationOnField.x,point.locationOnField.y));

  Pose2D resultingCrossing;
  double resultingSimExp,resultingSimExpNoClass;
  CrossingClass resultingClass;
  double distanceTrust = 1.0/300;
  double orientationTrust = 1.0 / (0.18 * pi);
  MODIFY("NDLineCrossingsTable:CrossingMapping:distanceTrust", distanceTrust);
  MODIFY("NDLineCrossingsTable:CrossingMapping:orientationTrust", orientationTrust);
  getClosestPoint(testPose,point, distanceTrust, orientationTrust, resultingCrossing, resultingClass, resultingSimExp, resultingSimExpNoClass);
  
  const LineCrossingsPercept::LineCrossing & c(point);
  const Vector2<int>& pf = c.locationOnField;
  const double & af = c.angleOnField;

  // draw characteristics of the sides: white->lineOnThisSide, black->noLineOnThisSide, light_gray->dontKnow
  LARGE_DOT("NDLineCrossingsTable:drawTestCrossingMapping", pf.x, pf.y, ColorClasses::red, ColorClasses::red);
  CIRCLE("NDLineCrossingsTable:drawTestCrossingMapping",pf.x,pf.y,60,15,Drawings::ps_solid,ColorClasses::red, Drawings::bs_null, ColorClasses::noColor);
  LINE("NDLineCrossingsTable:drawTestCrossingMapping", // side1
    pf.x,
    pf.y,
    pf.x + (int)(200.0*cos(af)),
    pf.y + (int)(200.0*sin(af)),
    30,
    Drawings::ps_solid,
    LineCrossingsPercept::crossingColors[c.side[0]]
  );
  LINE("NDLineCrossingsTable:drawTestCrossingMapping",// side2
    pf.x,
    pf.y,
    pf.x + (int)(200.0*cos(af+pi_2)),
    pf.y + (int)(200.0*sin(af+pi_2)),
    30,
    Drawings::ps_solid,
    LineCrossingsPercept::crossingColors[c.side[1]]
  );
  LINE("NDLineCrossingsTable:drawTestCrossingMapping",// side3
    pf.x,
    pf.y,
    pf.x + (int)(200.0*cos(af+pi)),
    pf.y + (int)(200.0*sin(af+pi)),
    30,
    Drawings::ps_solid,
    LineCrossingsPercept::crossingColors[c.side[2]]
  );
  LINE("NDLineCrossingsTable:drawTestCrossingMapping",// side4
    pf.x,
    pf.y,
    pf.x + (int)(200.0*cos(af+pi3_2)),
    pf.y + (int)(200.0*sin(af+pi3_2)),
    30,
    Drawings::ps_solid,
    LineCrossingsPercept::crossingColors[c.side[3]]
  );

  ARROW("NDLineCrossingsTable:drawTestCrossingMapping",
        pf.x,pf.y,
        resultingCrossing.translation.x,resultingCrossing.translation.y,
        30,Drawings::ps_solid,ColorRGBA(255,128,128));

}

void NDLineCrossingsTable::initVirtualCrossings(const FieldDimensions& fieldDimensions)
{

  // why were these removed? due to limited field of view, they should never be seen
  // addCrossing(virtualCrossing, 1, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftPenaltyArea);
  // addCrossing(virtualCrossing, 1, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightPenaltyArea);

  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftGroundline);
  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightGroundline);

  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftGroundline);
  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightGroundline);
}

void NDLineCrossingsTable::initTCrossings(const FieldDimensions& fieldDimensions)
{
  addCrossing(tCrossing, 3, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(tCrossing, 3, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightPenaltyArea);

  addCrossing(tCrossing, 2, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftGroundline);
  addCrossing(tCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightGroundline);

  addCrossing(tCrossing, 1, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(tCrossing, 1, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightPenaltyArea);
}

void NDLineCrossingsTable::initLCrossings(const FieldDimensions& fieldDimensions)
{
  addCrossing(lCrossing, 3, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftGroundline);
  addCrossing(lCrossing, 0, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightGroundline);
  addCrossing(lCrossing, 2, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftGroundline);
  addCrossing(lCrossing, 1, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightGroundline);

  addCrossing(lCrossing, 2, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(lCrossing, 1, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightPenaltyArea);
  addCrossing(lCrossing, 3, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(lCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightPenaltyArea);
}



void NDLineCrossingsTable::initXCrossings(const FieldDimensions& fieldDimensions)
{// they should be almost never seen (center circle filtering)
  addCrossing(xCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.centerCircleRadius);
  addCrossing(xCrossing, 0, fieldDimensions.xPosHalfWayLine, -fieldDimensions.centerCircleRadius);
}

void NDLineCrossingsTable::initMappingAreas(const FieldDimensions& fieldDimensions)
{
  /*  
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

  // define area thresholds (heuristics, not ML!)

  // decide distance between middle line crossings and penalty&groundline crossings
  // xAreaThreshold = middle of xPosHalfWayLine and (middle of groundLine and penaltyArea)
  xAreaThreshold = (fieldDimensions.xPosOpponentGroundline + fieldDimensions.xPosOpponentPenaltyArea) / 4;
  
  // decide distance between crossings on left ground line those on right penalty line (->between area 0 and 1)
  yAreaThreshold = (fieldDimensions.yPosLeftGroundline + fieldDimensions.yPosRightPenaltyArea) / 2;


  /*
  crossing characteristics sides:
     1
     |
  2--X--0
     |
     3
  */

  // area 0
  numOfCrossingsInArea[0] = 4;
  // L (opp left outer)
  fillCrossingTemplate( crossingsInArea[0][0], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentGroundline,
                        fieldDimensions.yPosLeftGroundline,
                        0,0,1,1 );
  // T (opp left penalty)
  fillCrossingTemplate( crossingsInArea[0][1], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOpponentGroundline,
                        fieldDimensions.yPosLeftPenaltyArea,
                        0,1,1,1 );
  // V (opp left)
  fillCrossingTemplate( crossingsInArea[0][2], NDLineCrossingsTable::virtualCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosLeftGroundline,
                        1,0,1,0 );
  // L (opp left penalty)
  fillCrossingTemplate( crossingsInArea[0][3], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosLeftPenaltyArea,
                        1,0,0,1 );

  // area 1
  numOfCrossingsInArea[1] = 3;
  // T (opp left penalty)
  fillCrossingTemplate( crossingsInArea[1][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOpponentGroundline,
                        fieldDimensions.yPosLeftPenaltyArea,
                        0,1,1,1 );
  // L (opp left penalty)
  fillCrossingTemplate( crossingsInArea[1][1], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosLeftPenaltyArea,
                        1,0,0,1 );
  // L (opp right penalty)
  fillCrossingTemplate( crossingsInArea[1][2], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosRightPenaltyArea,
                        1,1,0,0 );

  // area 2
  numOfCrossingsInArea[2] = 3;
  // T (opp right penalty)
  fillCrossingTemplate( crossingsInArea[2][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOpponentGroundline,
                        fieldDimensions.yPosRightPenaltyArea,
                        0,1,1,1 );
  // L (opp left penalty)
  fillCrossingTemplate( crossingsInArea[2][1], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosLeftPenaltyArea,
                        1,0,0,1 );
  // L (opp right penalty)
  fillCrossingTemplate( crossingsInArea[2][2], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosRightPenaltyArea,
                        1,1,0,0 );

  // area 3
  numOfCrossingsInArea[3] = 4;
  // T (opp right penalty)
  fillCrossingTemplate( crossingsInArea[3][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOpponentGroundline,
                        fieldDimensions.yPosRightPenaltyArea,
                        0,1,1,1 );
  // L (opp right outer)
  fillCrossingTemplate( crossingsInArea[3][1], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentGroundline,
                        fieldDimensions.yPosRightGroundline,
                        0,1,1,0 );
  // L (opp right penalty)
  fillCrossingTemplate( crossingsInArea[3][2], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosRightPenaltyArea,
                        1,1,0,0 );
  // V (opp right)
  fillCrossingTemplate( crossingsInArea[3][3], NDLineCrossingsTable::virtualCrossing,
                        fieldDimensions.xPosOpponentPenaltyArea,
                        fieldDimensions.yPosRightGroundline,
                        1,0,1,0 );

  // area 4
  numOfCrossingsInArea[4] = 2;
  // T (left middle)
  fillCrossingTemplate( crossingsInArea[4][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosHalfWayLine,
                        fieldDimensions.yPosLeftGroundline,
                        1,0,1,1 );
  // X (left)
  fillCrossingTemplate( crossingsInArea[4][1], NDLineCrossingsTable::xCrossing,
                        fieldDimensions.xPosHalfWayLine,
                        fieldDimensions.centerCircleRadius,
                        1,1,1,1 );

  // area 5
  numOfCrossingsInArea[5] = 2;
  // T (right middle)
  fillCrossingTemplate( crossingsInArea[5][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosHalfWayLine,
                        fieldDimensions.yPosRightGroundline,
                        1,1,1,0 );
  // X (right)
  fillCrossingTemplate( crossingsInArea[5][1], NDLineCrossingsTable::xCrossing,
                        fieldDimensions.xPosHalfWayLine,
                        - fieldDimensions.centerCircleRadius,
                        1,1,1,1 );

  // area 6
  numOfCrossingsInArea[6] = 4;
  // L (own left outer)
  fillCrossingTemplate( crossingsInArea[6][0],  NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnGroundline,
                        fieldDimensions.yPosLeftGroundline,
                        1,0,0,1 );
  // T (own left penalty)
  fillCrossingTemplate( crossingsInArea[6][1], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOwnGroundline,
                        fieldDimensions.yPosLeftPenaltyArea,
                        1,1,0,1 );
  // V (own left)
  fillCrossingTemplate( crossingsInArea[6][2], NDLineCrossingsTable::virtualCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosLeftGroundline,
                        1,0,1,0 );
  // L (own left penalty)
  fillCrossingTemplate( crossingsInArea[6][3], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosLeftPenaltyArea,
                        0,0,1,1 );

  // area 7
  numOfCrossingsInArea[7] = 3;
  // T (own left penalty)
  fillCrossingTemplate( crossingsInArea[7][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOwnGroundline,
                        fieldDimensions.yPosLeftPenaltyArea,
                        1,1,0,1 );
  // L (own left penalty)
  fillCrossingTemplate( crossingsInArea[7][1], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosLeftPenaltyArea,
                        0,0,1,1 );
  // L (own right penalty)
  fillCrossingTemplate( crossingsInArea[7][2], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosRightPenaltyArea,
                        0,1,1,0 );

  // area 8
  numOfCrossingsInArea[8] = 3;
  // T (own right penalty)
  fillCrossingTemplate( crossingsInArea[8][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOwnGroundline,
                        fieldDimensions.yPosRightPenaltyArea,
                        1,1,0,1 );
  // L (own left penalty)
  fillCrossingTemplate( crossingsInArea[8][1], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosLeftPenaltyArea,
                        0,0,1,1 );
  // L (own right penalty)
  fillCrossingTemplate( crossingsInArea[8][2], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosRightPenaltyArea,
                        0,1,1,0 );

  // area 9
  numOfCrossingsInArea[9] = 4;
  // T (own right penalty)
  fillCrossingTemplate( crossingsInArea[9][0], NDLineCrossingsTable::tCrossing,
                        fieldDimensions.xPosOwnGroundline,
                        fieldDimensions.yPosRightPenaltyArea,
                        1,1,0,1 );
  // L (own right outer)
  fillCrossingTemplate( crossingsInArea[9][1], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnGroundline,
                        fieldDimensions.yPosRightGroundline,
                        1,1,0,0 );
  // L (own right penalty)
  fillCrossingTemplate( crossingsInArea[9][2], NDLineCrossingsTable::lCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosRightPenaltyArea,
                        0,1,1,0 );
  // V (own right)
  fillCrossingTemplate( crossingsInArea[9][3], NDLineCrossingsTable::virtualCrossing,
                        fieldDimensions.xPosOwnPenaltyArea,
                        fieldDimensions.yPosRightGroundline,
                        1,0,1,0 );
}


bool NDLineCrossingsTable::getClosestPoint(
                          const Pose2D& crossingPoseInAbsoluteCoords,
                          const LineCrossingsPercept::LineCrossing& crossingPoint,
                          const double & distanceTrust,
                          const double & orientationTrust,
                          Pose2D & resultingCrossingPose,
                          CrossingClass & resultingCrossingClass,
                          double & resultingSimilarityExponent,
                          double & resultingSimilarityExponentWithoutClassification) const
{
  int whichArea = 0;

  const double & x = crossingPoseInAbsoluteCoords.translation.x;
  const double & y = crossingPoseInAbsoluteCoords.translation.y;
  double  absX = abs(x),
          absY = abs(y);

  // decide area
  if (absX < xAreaThreshold) // near the middle
  {
    if (y > 0)
    {
      whichArea = 4;
    }
    else
    {
      whichArea = 5;
    }
  }
  else // near one of the goals 
  {
    if (absY < yAreaThreshold) // in the middle before a goal
    {
      if (x>0) // opp half
      {
        if (y>0) // left
        {
          whichArea = 1;
        }
        else // right
        {
          whichArea = 2;
        }
      }
      else // own half
      {
        if (y>0) // left
        {
          whichArea = 7;
        }
        else // right
        {
          whichArea = 8;
        }
      }
    }
    else // closer to a field corner
    {
      if (x>0) // opp half
      {
        if (y>0) // left
        {
          whichArea = 0;
        }
        else // right
        {
          whichArea = 3;
        }
      }
      else // own half
      {
        if (y>0) // left
        {
          whichArea = 6;
        }
        else // right
        {
          whichArea = 9;
        }
      }
    }
  }

  // test similarity with all crossings in the area
  // calculate the exponent only, so no multiplication with the probability density gausians but adding only

  // Note: taking the distance is a further approximation, correct would be the angle differences as seen from the robot
  double distancePenalty,orientationPenalty,classificationPenalty;


  double resultingExponent,resultingExponentWithoutClassification;
  double bestExponent = 1000000;

  // we compare the 2 closest orientations
  int mappedOrientation = ((int) floor(crossingPoseInAbsoluteCoords.rotation / pi_2)) & 3;

  for (int j=0; j<=1; j++)
  {
    mappedOrientation = (mappedOrientation + j) % 4;
    
    // Note: orientationError is the same for all crossings (for the same rounding to 90°)
    orientationPenalty = abs( normalize( mappedOrientation*pi_2 - crossingPoseInAbsoluteCoords.rotation ) );
    orientationPenalty *= orientationTrust;

    for (int i=0; i<numOfCrossingsInArea[whichArea]; i++)
    {
      distancePenalty = (crossingsInArea[whichArea][i].positionOnField - crossingPoseInAbsoluteCoords.translation).abs();
      distancePenalty *= distanceTrust;

      classificationPenalty = 0;
      for (int n=0; n<4; n++)
      {
        classificationPenalty += classificationPenaltyTable[ crossingPoint.side[n] ][ crossingsInArea[whichArea][i].side[ (mappedOrientation+n)%4 ] ];
      }

      resultingExponent = orientationPenalty + distancePenalty + classificationPenalty;
      resultingExponentWithoutClassification = orientationPenalty + distancePenalty;
      if (resultingExponent < bestExponent)
      {
        bestExponent = resultingExponent;
        resultingSimilarityExponentWithoutClassification = resultingExponentWithoutClassification;
        resultingCrossingPose.translation.x = crossingsInArea[whichArea][i].positionOnField.x;
        resultingCrossingPose.translation.y = crossingsInArea[whichArea][i].positionOnField.y;
        resultingCrossingPose.rotation = mappedOrientation; // needs to be multiplied with pi_2 and normalized!
        resultingCrossingClass = crossingsInArea[whichArea][i].crossingClass;
      }
    }
  }

  resultingCrossingPose.rotation = normalize(resultingCrossingPose.rotation * pi_2);

  resultingSimilarityExponent = bestExponent;
  
  if (bestExponent > 999999)
  {
    // there are always at least 2 crossing in each area, so this should not happen! 
    // (but might if trusts are stupid.)
    return false;
  }


  return true;
}

bool NDLineCrossingsTable::getTemplatesFromSingleCrossing(const LineCrossingsPercept::LineCrossing& crossingPoint,
                                      std::vector<SampleTemplate> & templates) const
{
  return false;
}


/**
 * \brief berechnet mögliche Differenzen zwischen side[0] und der orientierungsgebenden Seite der Kreuzung
 *
 * crossing.angleOnField wird gegen side[0] gemessen, aber die orientierung der Kreuzung auf der Weltkarte
 * kann 0, +-pi_2 oder pi sein.
 * Rückgabewerte sind Vielfache von pi_2
 */
std::list<double> NDLineCrossingsTable::getPossibleOriOffsets(const LineCrossingsPercept::LineCrossing& crossing, CrossingClass& crossingClass){
  std::list<double> result;

  //für alle gilt: Falls alle seiten unbekannt - alles möglich
  //Für xCrossings auch
  if  ((crossingClass == xCrossing) ||
      (     crossing.side[0] == LineCrossingsPercept::dontKnow
        &&  crossing.side[1] == LineCrossingsPercept::dontKnow
        &&  crossing.side[2] == LineCrossingsPercept::dontKnow
        &&  crossing.side[3] == LineCrossingsPercept::dontKnow
      )) {
    result.push_back(0);
    result.push_back(pi_2);
    result.push_back(-pi_2);
    result.push_back(pi);
    return result;
  }

  switch (crossingClass) {
    case tCrossing:
      //orientierungsgebende seite ist erste nach einer nichtseite, gefolgt von drei seiten
      for (int i=0; i<4; i++) {
        if (    (crossing.side[i] == LineCrossingsPercept::noLineOnThisSide || crossing.side[i] == LineCrossingsPercept::dontKnow) 
            &&  (crossing.side[(i+1)%4] == LineCrossingsPercept::lineOnThisSide || crossing.side[(i+1)%4] == LineCrossingsPercept::dontKnow)
            &&  (crossing.side[(i+2)%4] == LineCrossingsPercept::lineOnThisSide || crossing.side[(i+2)%4] == LineCrossingsPercept::dontKnow)
            &&  (crossing.side[(i+3)%4] == LineCrossingsPercept::lineOnThisSide || crossing.side[(i+3)%4] == LineCrossingsPercept::dontKnow)
                ) 
        {
          result.push_back(pi_2*(i+1));          
        }
      }
      break;
    case lCrossing:
      //orientierungsgebende seite ist erste nach zwei nichtseiten, gefolgt von zwei seiten
      for (int i=0; i<4; i++) {
        if (    (crossing.side[i] == LineCrossingsPercept::noLineOnThisSide || crossing.side[i] == LineCrossingsPercept::dontKnow) 
            &&  (crossing.side[(i+1)%4] == LineCrossingsPercept::noLineOnThisSide || crossing.side[(i+1)%4] == LineCrossingsPercept::dontKnow)
            &&  (crossing.side[(i+2)%4] == LineCrossingsPercept::lineOnThisSide || crossing.side[(i+2)%4] == LineCrossingsPercept::dontKnow)
            &&  (crossing.side[(i+3)%4] == LineCrossingsPercept::lineOnThisSide || crossing.side[(i+3)%4] == LineCrossingsPercept::dontKnow)
                ) 
        {
          result.push_back(pi_2*(i+2));          
        }
      }
      break;
    case virtualCrossing:
      //orientierungsgebende seite ist eine der zwei seiten
      for (int i=0; i<4; i++) {
        if (    (crossing.side[i] == LineCrossingsPercept::lineOnThisSide || crossing.side[i] == LineCrossingsPercept::dontKnow) 
            &&  (crossing.side[(i+1)%4] == LineCrossingsPercept::noLineOnThisSide || crossing.side[(i+1)%4] == LineCrossingsPercept::dontKnow)
            &&  (crossing.side[(i+2)%4] == LineCrossingsPercept::lineOnThisSide || crossing.side[(i+2)%4] == LineCrossingsPercept::dontKnow)
            &&  (crossing.side[(i+3)%4] == LineCrossingsPercept::noLineOnThisSide || crossing.side[(i+3)%4] == LineCrossingsPercept::dontKnow)
                ) 
        {
          result.push_back(pi_2 * i);          
          result.push_back(pi_2 * (i+2));
          break; //symmetrisch
        }
      }
      break;
    default: 
      //Andere Kreuzungstypen werden nicht behandelt
      result.push_back(0);
  }

  return result;
}
