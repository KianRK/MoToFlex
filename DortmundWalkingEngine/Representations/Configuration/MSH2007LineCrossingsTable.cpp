/*
* position and type of line crossings on the field
*
* @author <A href=mailto:c_rohde@web.de>Carsten Rohde</A>
* @author <A href=mailto:judith-winter@web.de>Judith Winter</A>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
* @author Max Risler
*/


#include "MSH2007LineCrossingsTable.h"

int MSH2007LineCrossingsTable::NO_POINT_DISTANCE = 100000;

MSH2007LineCrossingsTable::MSH2007LineCrossingsTable()
{
  for(int i = 0; i < numOfCrossingClasses; i++)
    for(int j = 0; j < 4; j++)
      numOfCrossings[i][j] = 0;
}

void MSH2007LineCrossingsTable::initialize(const FieldDimensions& field)
{
  initVirtualCrossings(field);
  initTCrossings(field);
  initLCrossings(field);
  initXCrossings(field); 
  initFalseCrossings(field);
}

void MSH2007LineCrossingsTable::draw() const
{
  DECLARE_DEBUG_DRAWING("gt05-sl:crossings table", "drawingOnField");
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
          side1 = LineCrossingsPercept::lineOnThisSide;
          side2 = LineCrossingsPercept::lineOnThisSide;
          side3 = LineCrossingsPercept::lineOnThisSide;
          side4 = LineCrossingsPercept::lineOnThisSide;
          break;
        //case outerLCrossing:
        //  side1 = LineCrossingsPercept::borderOnThisSide;
        //  side2 = LineCrossingsPercept::borderOnThisSide;
        //  side3 = LineCrossingsPercept::noBorderOnThisSide;
        //  side4 = LineCrossingsPercept::noBorderOnThisSide;
        //  break;
        //case outerTCrossing: 
        //  side1 = LineCrossingsPercept::borderOnThisSide;
        //  side2 = LineCrossingsPercept::borderOnThisSide;
        //  side3 = LineCrossingsPercept::borderOnThisSide;
        //  side4 = LineCrossingsPercept::noBorderOnThisSide;
        //  break;
        //case outerVirtualCrossing:
        //  side1 = LineCrossingsPercept::borderOnThisSide;
        //  side2 = LineCrossingsPercept::noLineOnThisSide;
        //  side3 = LineCrossingsPercept::borderOnThisSide;
        //  side4 = LineCrossingsPercept::noLineOnThisSide;
        //  break;
        case falseCrossing: default:
          side1 = LineCrossingsPercept::noLineOnThisSide;
          side2 = LineCrossingsPercept::noLineOnThisSide;
          side3 = LineCrossingsPercept::noLineOnThisSide;
          side4 = LineCrossingsPercept::noLineOnThisSide;
          break;
        };

        COMPLEX_DRAWING("gt05-sl:crossings table",
        {
          Vector2<double> intersectionOnFieldGlobal = crossings[i][j][n];
          double angleOnFieldGlobal = j*pi_2;

          CIRCLE("gt05-sl:crossings table",intersectionOnFieldGlobal.x,intersectionOnFieldGlobal.y,60,6,Drawings::ps_solid,ColorClasses::red,Drawings::bs_null,ColorClasses::red);


          // draw characteristics of the sides: white->lineOnThisSide, black->noLineOnThisSide, light_gray->dontKnow
          LINE("gt05-sl:crossings table", // side1
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side1]
          );
          LINE("gt05-sl:crossings table",// side2
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal+pi_2)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal+pi_2)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side2]
          );
          LINE("gt05-sl:crossings table",// side3
            intersectionOnFieldGlobal.x,
            intersectionOnFieldGlobal.y,
            intersectionOnFieldGlobal.x + (int)(200.0*cos(angleOnFieldGlobal+pi)),
            intersectionOnFieldGlobal.y + (int)(200.0*sin(angleOnFieldGlobal+pi)),
            7,
            Drawings::ps_solid,
            LineCrossingsPercept::crossingColors[side3]
          );
          LINE("gt05-sl:crossings table",// side4
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

void MSH2007LineCrossingsTable::initVirtualCrossings(const FieldDimensions& fieldDimensions)
{

  // why were these removed? due to limited field of view, they should never be seen
  // addCrossing(virtualCrossing, 1, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftPenaltyArea);
  // addCrossing(virtualCrossing, 1, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightPenaltyArea);

  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftGroundline);
  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightGroundline);

  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftGroundline);
  addCrossing(virtualCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightGroundline);

  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightFieldBorder);

  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightFieldBorder);

  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightFieldBorder);

  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightFieldBorder);

  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerVirtualCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightFieldBorder);

  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOpponentFieldBorder, fieldDimensions.yPosLeftGroundline);
  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOpponentFieldBorder, fieldDimensions.yPosRightGroundline);

  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOwnFieldBorder, fieldDimensions.yPosLeftGroundline);
  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOwnFieldBorder, fieldDimensions.yPosRightGroundline);

  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOpponentFieldBorder, fieldDimensions.yPosLeftPenaltyArea);
  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOpponentFieldBorder, fieldDimensions.yPosRightPenaltyArea);

  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOwnFieldBorder, fieldDimensions.yPosLeftPenaltyArea);
  //addCrossing(outerVirtualCrossing, 1, fieldDimensions.xPosOwnFieldBorder, fieldDimensions.yPosRightPenaltyArea);
}

void MSH2007LineCrossingsTable::initTCrossings(const FieldDimensions& fieldDimensions)
{
  addCrossing(tCrossing, 3, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(tCrossing, 3, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightPenaltyArea);

  addCrossing(tCrossing, 2, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftGroundline);
  addCrossing(tCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightGroundline);

  addCrossing(tCrossing, 1, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(tCrossing, 1, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightPenaltyArea);
}

void MSH2007LineCrossingsTable::initLCrossings(const FieldDimensions& fieldDimensions)
{
  addCrossing(lCrossing, 3, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosLeftGroundline);
  addCrossing(lCrossing, 0, fieldDimensions.xPosOwnGroundline, fieldDimensions.yPosRightGroundline);
  addCrossing(lCrossing, 2, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosLeftGroundline);
  addCrossing(lCrossing, 1, fieldDimensions.xPosOpponentGroundline, fieldDimensions.yPosRightGroundline);

  addCrossing(lCrossing, 2, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(lCrossing, 1, fieldDimensions.xPosOwnPenaltyArea, fieldDimensions.yPosRightPenaltyArea);
  addCrossing(lCrossing, 3, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(lCrossing, 0, fieldDimensions.xPosOpponentPenaltyArea, fieldDimensions.yPosRightPenaltyArea);

  //addCrossing(outerLCrossing, 3, fieldDimensions.xPosOwnFieldBorder, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerLCrossing, 0, fieldDimensions.xPosOwnFieldBorder, fieldDimensions.yPosRightFieldBorder);
  //addCrossing(outerLCrossing, 2, fieldDimensions.xPosOpponentFieldBorder, fieldDimensions.yPosLeftFieldBorder);
  //addCrossing(outerLCrossing, 1, fieldDimensions.xPosOpponentFieldBorder, fieldDimensions.yPosRightFieldBorder);
}

void MSH2007LineCrossingsTable::initFalseCrossings(const FieldDimensions& fieldDimensions)
{ // should be the virtual crossings which can never be seen
  addCrossing(falseCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosLeftPenaltyArea);
  addCrossing(falseCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.yPosRightPenaltyArea);
}

void MSH2007LineCrossingsTable::initXCrossings(const FieldDimensions& fieldDimensions)
{// they should be almost never seen (center circle filtering)
  addCrossing(xCrossing, 0, fieldDimensions.xPosHalfWayLine, fieldDimensions.centerCircleRadius);
  addCrossing(xCrossing, 0, fieldDimensions.xPosHalfWayLine, -fieldDimensions.centerCircleRadius);
}

/**
 * \brief berechnet mögliche Differenzen zwischen side[0] und der orientierungsgebenden Seite der Kreuzung
 *
 * crossing.angleOnField wird gegen side[0] gemessen, aber die orientierung der Kreuzung auf der Weltkarte
 * kann 0, +-pi_2 oder pi sein.
 * Rückgabewerte sind Vielfache von pi_2
 */
std::list<double> MSH2007LineCrossingsTable::getPossibleOriOffsets(const LineCrossingsPercept::LineCrossing& crossing, CrossingClass& crossingClass){
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


Vector2<double> MSH2007LineCrossingsTable::getClosestPoint(const Vector2<double>& point, CrossingClass crossingClass, int orientation) const
{
  //Vector2<double> minimum(NO_POINT_DISTANCE,NO_POINT_DISTANCE);
  Pose2D minimum(NO_POINT_DISTANCE, NO_POINT_DISTANCE);
  double minDist = NO_POINT_DISTANCE;
  double maxAngleDelta;
  updateClosestPoint(minimum, maxAngleDelta, false, minDist, point, crossingClass, orientation, crossingClass);
  return minimum.translation;
}

//void MSH2007LineCrossingsTable::updateClosestPoint(Vector2<double>& minimum, double& minDist, const Vector2<double>& point, CrossingClass crossingClass, int orientation, CrossingClass& minCrossingClass) const
//{
//  Pose2D temp;
//  updateClosestPoint(temp, minDist, point, crossingClass, orientation, minCrossingClass);
//  minimum = temp.translation;
//}

void MSH2007LineCrossingsTable::updateClosestPoint(Pose2D& result, double& maxAngleDelta, bool characterized, double& minDist, const Vector2<double>& point, CrossingClass crossingClass, int orientation, CrossingClass& minCrossingClass) const
{
  for(int i = 0; i < numOfCrossings[crossingClass][orientation]; i++)
  {
    Vector2<double> diff = point - crossings[crossingClass][orientation][i];
    if (diff.abs()<minDist)
    {
      minDist = diff.abs();
      result.translation = crossings[crossingClass][orientation][i];
      if (!characterized)
      {
        maxAngleDelta = pi_2;
        //result.rotation = 0; // ? shouldn't this be the closest orientation, i.e. orientation*pi_2 ?
        result.rotation = orientation*pi_2;
      }
      else
        switch (crossingClass) 
      {
        case lCrossing:
        case tCrossing:
        case outerLCrossing:
          maxAngleDelta = pi_2; 
          result.rotation = orientation*pi_2;
          break;
          //case outerTCrossing, //unused, they dont exist on the field
        case virtualCrossing:
        case outerVirtualCrossing:
          maxAngleDelta = pi_2;
          result.rotation = orientation*pi_2;
          break;
        case xCrossing:
        case falseCrossing:
        default:
          maxAngleDelta = pi_2;
          result.rotation = 0;
          break;

      }
      minCrossingClass = crossingClass;
    }
  }
}

Pose2D MSH2007LineCrossingsTable::getClosestPoint(const Pose2D& pose, double& maxAngleDelta,
                                                  const LineCrossingsPercept::LineCrossing& crossingPoint, 
                                                  CrossingClass& crossingClass) const
{
  //Vector2<double> minimum(NO_POINT_DISTANCE,NO_POINT_DISTANCE);
  Pose2D minimum(NO_POINT_DISTANCE, NO_POINT_DISTANCE);
  double minDist = NO_POINT_DISTANCE;

  double poseAngle = normalize(pose.rotation + pi_4);
  if (poseAngle < 0) poseAngle += pi2;
  int poseOrientation = (int) (poseAngle / pi_2);
  bool stronglyCharacterized = false;
  //if (!crossingPoint.borderLine)
  {
    if (crossingPoint.side[0] == LineCrossingsPercept::lineOnThisSide &&
      crossingPoint.side[3] == LineCrossingsPercept::noLineOnThisSide)
    { // possible L or T, 0 orientation
      if (crossingPoint.side[1] >= LineCrossingsPercept::dontKnow)
      {
        if (crossingPoint.side[2] < LineCrossingsPercept::lineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, lCrossing, poseOrientation, crossingClass);
          stronglyCharacterized = true;
        }
        if (crossingPoint.side[2] > LineCrossingsPercept::noLineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, tCrossing, poseOrientation, crossingClass);
          stronglyCharacterized = true;
        }
      }
    }
    if (crossingPoint.side[1] == LineCrossingsPercept::lineOnThisSide &&
      crossingPoint.side[0] == LineCrossingsPercept::noLineOnThisSide)
    { // possible L or T, 1 orientation
      if (crossingPoint.side[2] >= LineCrossingsPercept::dontKnow)
      {
        if (crossingPoint.side[3] < LineCrossingsPercept::lineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, lCrossing, (poseOrientation + 1) % 4, crossingClass);
          stronglyCharacterized = true;
        }
        if (crossingPoint.side[3] > LineCrossingsPercept::noLineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, tCrossing, (poseOrientation + 1) % 4, crossingClass);
          stronglyCharacterized = true;
        }
      }
    }
    if (crossingPoint.side[2] == LineCrossingsPercept::lineOnThisSide &&
      crossingPoint.side[1] == LineCrossingsPercept::noLineOnThisSide)
    { // possible L or T, 2 orientation
      if (crossingPoint.side[3] >= LineCrossingsPercept::dontKnow)
      {
        if (crossingPoint.side[0] < LineCrossingsPercept::lineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, lCrossing, (poseOrientation + 2) % 4, crossingClass);
          stronglyCharacterized = true;
        }
        if (crossingPoint.side[0] > LineCrossingsPercept::noLineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, tCrossing, (poseOrientation + 2) % 4, crossingClass);
          stronglyCharacterized = true;
        }
      }
    }
    if (crossingPoint.side[3] == LineCrossingsPercept::lineOnThisSide &&
      crossingPoint.side[2] == LineCrossingsPercept::noLineOnThisSide)
    { // possible L or T, 3 orientation
      if (crossingPoint.side[0] >= LineCrossingsPercept::dontKnow)
      {
        if (crossingPoint.side[1] < LineCrossingsPercept::lineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, lCrossing, (poseOrientation + 3) % 4, crossingClass);
          stronglyCharacterized = true;
        }
        if (crossingPoint.side[1] > LineCrossingsPercept::noLineOnThisSide)
        {
          updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, tCrossing, (poseOrientation + 3) % 4, crossingClass);
          stronglyCharacterized = true;
        }
      }
    }
    if (crossingPoint.side[1] == LineCrossingsPercept::noLineOnThisSide &&
      crossingPoint.side[3] == LineCrossingsPercept::noLineOnThisSide &&
      crossingPoint.side[0] >= LineCrossingsPercept::dontKnow &&
      crossingPoint.side[2] >= LineCrossingsPercept::dontKnow)
    {
      // possibly virtual, orientation 0
      updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, virtualCrossing, poseOrientation % 2, crossingClass);
      stronglyCharacterized = true;
    }
    if (crossingPoint.side[0] == LineCrossingsPercept::noLineOnThisSide &&
      crossingPoint.side[2] == LineCrossingsPercept::noLineOnThisSide &&
      crossingPoint.side[1] >= LineCrossingsPercept::dontKnow &&
      crossingPoint.side[3] >= LineCrossingsPercept::dontKnow)
    {
      // possibly virtual, orientation 1
      updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, virtualCrossing, (poseOrientation + 1) % 2, crossingClass);
      stronglyCharacterized = true;
    }
  }
  //if (!crossingPoint.fieldLine)
  //{
  //  if ((crossingPoint.side[0] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[1] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[2] <= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[3] <= LineCrossingsPercept::borderDontKnow)
  //    ||
  //    (crossingPoint.side[0] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[1] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[2] == LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[3] == LineCrossingsPercept::noBorderOnThisSide))
  //  {
  //    // possibly outer L, orientation 0
  //    updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, outerLCrossing, poseOrientation, crossingClass);
  //    stronglyCharacterized = true;
  //  }
  //  if ((crossingPoint.side[1] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[2] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[0] <= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[3] <= LineCrossingsPercept::borderDontKnow)
  //    ||
  //    (crossingPoint.side[1] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[2] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[0] == LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[3] == LineCrossingsPercept::noBorderOnThisSide))
  //  {
  //    // possibly outer L, orientation 1
  //    updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, outerLCrossing, (poseOrientation + 1) % 4, crossingClass);
  //    stronglyCharacterized = true;
  //  }
  //  if ((crossingPoint.side[2] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[3] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[0] <= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[1] <= LineCrossingsPercept::borderDontKnow)
  //    ||
  //    (crossingPoint.side[2] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[3] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[0] == LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[1] == LineCrossingsPercept::noBorderOnThisSide))
  //  {
  //    // possibly outer L, orientation 2
  //    updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, outerLCrossing, (poseOrientation + 2) % 4, crossingClass);
  //    stronglyCharacterized = true;
  //  }
  //  if ((crossingPoint.side[3] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[0] == LineCrossingsPercept::borderOnThisSide &&
  //    crossingPoint.side[1] <= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[2] <= LineCrossingsPercept::borderDontKnow)
  //    ||
  //    (crossingPoint.side[3] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[0] >= LineCrossingsPercept::borderDontKnow &&
  //    crossingPoint.side[1] == LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[2] == LineCrossingsPercept::noBorderOnThisSide))
  //  {
  //    // possibly outer L, orientation 3
  //    updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, outerLCrossing, (poseOrientation + 3) % 4, crossingClass);
  //    stronglyCharacterized = true;
  //  }
  //}
  //if (crossingPoint.fieldLine && crossingPoint.borderLine)
  //{
  //  if (crossingPoint.side[0] >= LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[1] < LineCrossingsPercept::noBorderOnThisSide && 
  //    crossingPoint.side[2] >= LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[3] < LineCrossingsPercept::noBorderOnThisSide)
  //  {
  //    // possibly outer virtual, orientation 0
  //    updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, outerVirtualCrossing, poseOrientation % 2, crossingClass);
  //    stronglyCharacterized = true;
  //  }
  //  if (crossingPoint.side[0] < LineCrossingsPercept::noBorderOnThisSide && //this is a field line
  //    crossingPoint.side[1] >= LineCrossingsPercept::noBorderOnThisSide &&//this means, its a border
  //    crossingPoint.side[2] < LineCrossingsPercept::noBorderOnThisSide &&
  //    crossingPoint.side[3] >= LineCrossingsPercept::noBorderOnThisSide)
  //  {
  //    // possibly outer virtual, orientation 1
  //    updateClosestPoint(minimum, maxAngleDelta, true, minDist, pose.translation, outerVirtualCrossing, (poseOrientation +1 ) % 2, crossingClass);
  //    stronglyCharacterized = true;
  //  }
  //}
  if (!stronglyCharacterized)
  {
    //if (!crossingPoint.borderLine)
    {
      // inner L crossings: only field lines allowed
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::lineOnThisSide)))
        // possibly L, orientation 0
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, lCrossing, poseOrientation, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::lineOnThisSide)))
        // possibly L, orientation 1
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, lCrossing, (poseOrientation + 1) % 4, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly L, orientation 2
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, lCrossing, (poseOrientation + 2) % 4, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly L, orientation 3
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, lCrossing, (poseOrientation + 3) % 4, crossingClass);
      // inner T crossings: only field lines allowed
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::lineOnThisSide)))
        // possibly T, orientation 0
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, tCrossing, poseOrientation, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly T, orientation 1
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, tCrossing, (poseOrientation + 1) % 4, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly T, orientation 2
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, tCrossing, (poseOrientation + 2) % 4, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly T, orientation 3
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, tCrossing, (poseOrientation + 3) % 4, crossingClass);
      // inner virtual crossings: only field lines allowed
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::lineOnThisSide)))
        // possibly virtual, orientation 0
        // there are only two possible orientations for virtual crossings, so we use orientation % 2
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, virtualCrossing, poseOrientation % 2, crossingClass);
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::lineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly virtual, orientation 1
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, virtualCrossing, (poseOrientation + 1) % 2, crossingClass);
      // x crossing: only field lines allowed
      if ((crossingPoint.outOfImage ||
        (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
        crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide)))
        // possibly x
        updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, xCrossing, 0, crossingClass);
    }
    //if (crossingPoint.borderLine&&crossingPoint.fieldLine)
    //{ // Hmmm...
    //  // outer virtual crossings: must be the intersection of a field line and a border line
    //  if ((crossingPoint.outOfImage ||
    //    (crossingPoint.side[0] == LineCrossingsPercept::borderOnThisSide ||
    //    crossingPoint.side[0] == LineCrossingsPercept::borderDontKnow
    //    ) &&
    //    (crossingPoint.side[2] == LineCrossingsPercept::borderOnThisSide ||
    //    crossingPoint.side[2] == LineCrossingsPercept::borderDontKnow
    //    )))
    //    // possibly outer virtual, orientation 0
    //    updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, outerVirtualCrossing, poseOrientation % 2, crossingClass);
    //  if ((crossingPoint.outOfImage ||
    //    (crossingPoint.side[1] == LineCrossingsPercept::borderOnThisSide ||
    //    crossingPoint.side[1] == LineCrossingsPercept::borderDontKnow
    //    ) &&
    //    (crossingPoint.side[3] == LineCrossingsPercept::borderOnThisSide ||
    //    crossingPoint.side[3] == LineCrossingsPercept::borderDontKnow
    //    )))
    //    // possibly outer virtual, orientation 1
    //    updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, outerVirtualCrossing, (poseOrientation + 1) % 2, crossingClass);
    //}
    //if (!crossingPoint.fieldLine)
    //{
    //  // outer L crossings: only border lines allowed
    //  if ((crossingPoint.outOfImage ||
    //    (crossingPoint.side[0] != LineCrossingsPercept::noBorderOnThisSide &&
    //    crossingPoint.side[1] != LineCrossingsPercept::noBorderOnThisSide)))
    //    // possibly outer L, orientation 0
    //    updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, outerLCrossing, poseOrientation, crossingClass);
    //  if ((crossingPoint.outOfImage ||
    //    (crossingPoint.side[1] != LineCrossingsPercept::noBorderOnThisSide &&
    //    crossingPoint.side[2] != LineCrossingsPercept::noBorderOnThisSide)))
    //    // possibly outer L, orientation 1
    //    updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, outerLCrossing, (poseOrientation + 1) % 4, crossingClass);
    //  if ((crossingPoint.outOfImage ||
    //    (crossingPoint.side[2] != LineCrossingsPercept::noBorderOnThisSide &&
    //    crossingPoint.side[3] != LineCrossingsPercept::noBorderOnThisSide)))
    //    // possibly outer L, orientation 2
    //    updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, outerLCrossing, (poseOrientation + 2) % 4, crossingClass);
    //  if ((crossingPoint.outOfImage ||
    //    (crossingPoint.side[0] != LineCrossingsPercept::noBorderOnThisSide &&
    //    crossingPoint.side[3] != LineCrossingsPercept::noBorderOnThisSide)))
    //    // possibly outer L, orientation 3
    //    updateClosestPoint(minimum, maxAngleDelta, false, minDist, pose.translation, outerLCrossing, (poseOrientation + 3) % 4, crossingClass);
    //}

  }
  // on the real field, there's no such a thing as an outer T crossing!
  //if (crossingPoint.outOfImage ||
  //    (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide))
  //  // possibly outer T, orientation 0
  //  updateClosestPoint(minimum, maxAngleDelta, minDist, pose.translation, outerTCrossing, poseOrientation, crossingClass);
  //if (crossingPoint.outOfImage ||
  //    (crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide))
  //  // possibly outer T, orientation 1
  //  updateClosestPoint(minimum, maxAngleDelta, minDist, pose.translation, outerTCrossing, (poseOrientation + 1) % 4, crossingClass);
  //if (crossingPoint.outOfImage ||
  //    (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[2] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide))
  //  // possibly outer T, orientation 2
  //  updateClosestPoint(minimum, maxAngleDelta, minDist, pose.translation, outerTCrossing, (poseOrientation + 2) % 4, crossingClass);
  //if (crossingPoint.outOfImage ||
  //    (crossingPoint.side[0] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[1] != LineCrossingsPercept::noLineOnThisSide &&
  //     crossingPoint.side[3] != LineCrossingsPercept::noLineOnThisSide))
  //  // possibly outer T, orientation 3
  //  updateClosestPoint(minimum, maxAngleDelta, minDist, pose.translation, outerTCrossing, (poseOrientation + 3) % 4, crossingClass);
  // end of outer T crossing section
  return minimum;
}

//Vector2<double> MSH2007LineCrossingsTable::getClosestPoint(const Pose2D& pose,
//  const LineCrossingsPercept::LineCrossing& crossingPoint, 
//  CrossingClass& crossingClass) const
//{
//  Pose2D temp = getClosestPoint(pose, crossingPoint, crossingClass);
//  return temp.translation;
//}
