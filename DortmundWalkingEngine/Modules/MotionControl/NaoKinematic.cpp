/** 
* @file NaoKinematic.cpp
* This file implements the inverse kinematic.
* @author <A href="mailto:Stefan.Czarnetzki@uni-dortmund.de">Stefan Czarnetzki</A>
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a> 
*/

//#define LOGGING
#include "DortmundWalkingEngine/StepData.h"
#include "NaoKinematic.h"
#ifndef WALKING_SIMULATOR
#include "Platform/GTAssert.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Pose3D.h"
#include "Tools/Math/Common.h"
#include "Tools/Settings.h"
#include "../../Tools/Debugging/CSVLogger.h"
#else
#include "math/Pose3D.h"
#include "math/Common.h"
#include "CSVLogger.h"
#endif
NaoKinematic::NaoKinematic() 
{
}

NaoKinematic::~NaoKinematic() {

}



Vector3<double> NaoKinematic::checkConstraints(Vector3<double> lf, double lfr, Vector3<double> rf, double rfr, bool correctleft)
{
	Vector2<double> checkPoints[2], constraintPoint, innerPoint;
	Vector3<double> cf3D;

	checkPoints[0].x=theRobotDimensions.footFront;
	checkPoints[1].x=theRobotDimensions.footBack;
	constraintPoint.x=theRobotDimensions.footFront;
	innerPoint.x=theRobotDimensions.footFront;

	if (correctleft)
	{
		checkPoints[0].y=-theRobotDimensions.footInner;
		checkPoints[0].rotate(lfr);
		checkPoints[0].x+=lf.x;
		checkPoints[0].y+=lf.y;
		
		checkPoints[1].y=-theRobotDimensions.footInner;
		checkPoints[1].rotate(lfr);
		checkPoints[1].x+=lf.x;
		checkPoints[1].y+=lf.y;
		
		constraintPoint.y=theRobotDimensions.footInner;
		constraintPoint.rotate(rfr);
		constraintPoint.x+=rf.x;
		constraintPoint.y+=rf.y;

		innerPoint.y=-theRobotDimensions.footOuter;
		innerPoint.rotate(rfr);
		innerPoint.x+=rf.x;
		innerPoint.y+=rf.y;

		cf3D=lf;
	}
	else
	{
		// right foot will be checked and moved

		checkPoints[0].y=theRobotDimensions.footInner;
		checkPoints[0].rotate(rfr);
		checkPoints[0].x+=rf.x;
		checkPoints[0].y+=rf.y;
		
		checkPoints[1].y=theRobotDimensions.footInner;
		checkPoints[1].rotate(rfr);
		checkPoints[1].x+=rf.x;
		checkPoints[1].y+=rf.y;
		
		constraintPoint.y=-theRobotDimensions.footInner;
		constraintPoint.rotate(lfr);
		constraintPoint.x+=lf.x;
		constraintPoint.y+=lf.y;

		innerPoint.y=theRobotDimensions.footOuter;
		innerPoint.rotate(lfr);
		innerPoint.x+=lf.x;
		innerPoint.y+=lf.y;

		cf3D=rf;
	}

	Vector2<double> a=innerPoint-constraintPoint;
	Vector2<double> b[2];
	b[0]=checkPoints[0]-constraintPoint;
	b[1]=checkPoints[1]-constraintPoint;

	a.normalize();

	double dot[2];
	dot[0]=a.x*b[0].x+a.y*b[0].y;
	dot[1]=a.x*b[1].x+a.y*b[1].y;

	int largestIndex;
	if (dot[0]>dot[1])
		largestIndex=0;
	else
		largestIndex=1;

	Vector2<double> cf(cf3D.x, cf3D.y);
	
	if (dot[largestIndex]>0)
			cf-=a*dot[largestIndex];
	
	cf3D.x=cf.x;
	cf3D.y=cf.y;
	return cf3D; 
}


void NaoKinematic::calcLegJoints(JointData::Joint whichSideJoint0, const Vector3<double>& position, const Vector3<double>& rotation, double t0, JointRequest& jointRequest)
{
	double footHeight = theRobotDimensions.heightLeg5Joint;
	double tibiaLength = theRobotDimensions.lowerLegLength;
	//double thighLength = theRobotDimensions.upperLegLength;
	double hipOffsetY = 0.5 * theRobotDimensions.lengthBetweenLegs;
	double hipOffsetZ = 0;//85;

#ifdef WALKING_SIMULATOR
	hipOffsetZ = 85;
#endif

	double right = (whichSideJoint0 == JointData::legLeft0 ? -1 : 1);

	double xsign = 1;
	if (position.x < 0)
		xsign = -1;

	double tiltAngleOfFirstJointAxis =- pi / 2 + right * pi / 4;

	Matrix3x3<double> M1;
	M1[1][1] = cos(tiltAngleOfFirstJointAxis);
	M1[2][1] = -sin(tiltAngleOfFirstJointAxis);
	M1[1][2] = sin(tiltAngleOfFirstJointAxis);
	M1[2][2] = cos(tiltAngleOfFirstJointAxis);

	Matrix3x3<double> M2;
	M2[1][1] = cos(tiltAngleOfFirstJointAxis);
	M2[2][1] = sin(tiltAngleOfFirstJointAxis);
	M2[1][2] = -sin(tiltAngleOfFirstJointAxis);
	M2[2][2] = cos(tiltAngleOfFirstJointAxis);

	double rx=rotation.x;

	// BEGIN	footRotX=rotMatrixAroundX(right*rx);
	//			footRot=rotMatrixAroundY(ry);
	//			footHeightVec=footRot*footRotX*[0;0;46];
	RotationMatrix footRot, footRotX;
	Vector3<double> footHeightVec(0, 0, footHeight);
	footRotX.rotateX(right*rx);
	footRot.rotateY(rotation.y);
	footHeightVec = footRot * footRotX * footHeightVec;

	// END

	// BEGIN p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);
	RotationMatrix rotZ;
	rotZ.rotateZ(-t0);

	Vector3<double> p(position.x, position.y + right * hipOffsetY, position.z + hipOffsetZ);
	p = p + footHeightVec;
	p = M1 * rotZ * M2 * p;
	// END p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);

	double ysign = -1*right;

	double r = p.abs();
	if (r>200)
	{
		//OUTPUT(idText, text, "Bein zu kurz!\r\n");
		p.normalize(200);
		r=200;
		//Vector3<double>(0, 0, -hipOffsetZ) + M1 * rotZ * M2 * Vector3<double>(

	}

	double t3 = 2 * asin((r / 2) / tibiaLength);
	double t1 = atan(p[1]/p[2]);

	Vector3<double> p1(p);
	p1[0] = 0;
	double t2a = (pi-t3)/2;

	double soll;
	if (p[0]!=0)
		soll=atan(p[0]/p1.abs());
	else
		soll=0;

	double t2=t2a+soll;


	tiltAngleOfFirstJointAxis = - pi / 2 +  pi / 4;

	M1[1][1] = cos(tiltAngleOfFirstJointAxis);
	M1[2][1] = -sin(tiltAngleOfFirstJointAxis);
	M1[1][2] = sin(tiltAngleOfFirstJointAxis);
	M1[2][2] = cos(tiltAngleOfFirstJointAxis);

	M2[1][1] = cos(tiltAngleOfFirstJointAxis);
	M2[2][1] = sin(tiltAngleOfFirstJointAxis);
	M2[1][2] = -sin(tiltAngleOfFirstJointAxis);
	M2[2][2] = cos(tiltAngleOfFirstJointAxis);

	// BEGIN R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);
	Matrix3x3<double> R;
	RotationMatrix rotZ2, rotY4, rotX, rotY5;
	rotZ2.rotateZ(t0);
	rotY4.rotateY(-t2);
	rotX.rotateX(ysign * t1);
	rotY5.rotateY(pi - t3);
	R = M1 * rotZ2 * M2 * rotX * rotY4 * rotY5;
	// END R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);

	// BEGIN schnitt=cross(R*[0;1;0], [0;0;1]);
	Vector3<double> v3(0, 0, 1);
	Vector3<double> v4(0, 1, 0);
	v4 = R * v4;
	Vector3<double> schnitt(v4 ^ (footRot * v3));
	// END schnitt=cross(R*[0;1;0], [0;0;1]);


	Vector3<double> vek(R * v3);

	double t4 = -asin((schnitt * vek) / (schnitt.abs() * vek.abs()));

	// BEGIN R=R*rotMatrixAroundY(t4);
	RotationMatrix rotY6;
	rotY6.rotateY(t4);
	R = R * rotY6;
	// END R=R*rotMatrixAroundY(t4);

	// BEGIN schnitt=cross(R*[1;0;0], [0;0;1]);
	Vector3<double> v5(1, 0, 0);
	schnitt = (R * v5) ^ (footRot * v3);
	// END schnitt=cross(R*[1;0;0], [0;0;1]);

	vek = R * v3;

	// BEGIN t5=-asin((schnitt'*vek)/(norm(schnitt)*norm(vek)))+rx;
	double t5 = -asin((schnitt * vek) / (schnitt.abs() * vek.abs()))+right*rx;
	// END

	jointRequest.angles[whichSideJoint0 + 0] = t0; // [-90 ;  0 ] = [...
	jointRequest.angles[whichSideJoint0 + 1] = ysign * t1; // [-100; 25 ] = [...
	jointRequest.angles[whichSideJoint0 + 2] = -t2; // [-45 ; 25 ]
	jointRequest.angles[whichSideJoint0 + 3] = pi - t3; // [ 0  ; 130]
	jointRequest.angles[whichSideJoint0 + 4] = t4; // [-75 ; 45 ]
	jointRequest.angles[whichSideJoint0 + 5] = t5; // [-25 ; 45 ]	
}

void NaoKinematic::calcLegJoints(JointData::Joint whichSideJoint0, const Vector3<double>& position, const Vector3<double>& rotation, JointRequest& jointRequest)
{
	// to provide clearification about the constants in the robot dimensions:
	// (referring to the figure provided by aldebaran)
	// http://www.aldebaran-robotics.com/robocup/doc/docref/general/robocup/media/RobocupmDHDefintion.jpg
	//double footHeight =46;//= theRobotDimensions.heightLeg5Joint;
	//double tibiaLength =100;//= theRobotDimensions.lowerLegLength;
	//double thighLength =100;//= theRobotDimensions.upperLegLength;
	//double hipOffsetY =50;//= 0.5 * theRobotDimensions.lengthBetweenLegs;
	//// HipOffsetZ = ?
	//double hipOffsetZ = 0;//85;
  
	double footHeight = theRobotDimensions.heightLeg5Joint;
	double tibiaLength = theRobotDimensions.lowerLegLength;
	double thighLength = theRobotDimensions.upperLegLength;
	double hipOffsetY = 0.5 * theRobotDimensions.lengthBetweenLegs;
	double hipOffsetZ = 0;//85;

#ifdef WALKING_SIMULATOR
	hipOffsetZ = 85;
#endif

	double sign = (whichSideJoint0 == JointData::legLeft0 ? -1 : 1);

	RotationMatrix rot;
	rot.rotateX(rotation.x);
	rot.rotateY(rotation.y);
	rot.rotateZ(rotation.z);

	Pose3D T(rot,position);

	//    T = T(Waist->J0) T0to5 T(J5->Foot)
	// => T0to5 = T(Waist->J0)^-1 T T(J5->Foot)^-1
	// with M1 = T(Waist->J0)^-1
	//      M2 = T(J5->Foot)^-1

	Pose3D M1(0, - sign * hipOffsetY, - hipOffsetZ);
	M1 = M1.invert();
	Pose3D M2(0,0, - footHeight);
	M2 = M2.invert();

	Pose3D T0to5 = M1 * T * M2;


	// now let's calculate the angles
	double t0,t1,t2,t3,t4,t5;
	double angle1,angle2; // will be needed later

	double length = T0to5.translation.abs(); // sqrt( T0to5(1,4) *  T0to5(1,4) + T0to5(2,4) * T0to5(2,4) + T0to5(3,4) * T0to5(3,4) );

	if (length > thighLength + tibiaLength)
	{
		t3 = 0;
		angle1 = 0;
		angle2 = 0;
	}
	else
	{
		// first the knee angle by law of cosines
		double kneeAngle = acos( (tibiaLength*tibiaLength + thighLength*thighLength - length*length) / (2*thighLength*tibiaLength) );
		t3 = pi - kneeAngle; // Will be a positive number. t3 might theoretically also be negative, but the knee joint can only be bent in one direction, so this is the correct one.

		angle1 = asin( thighLength * sin(kneeAngle) / length); // law of sines
		angle2 = pi - kneeAngle - angle1;
	}


	Pose3D T0to5Inv = T0to5.invert();
	t5 = atan2( T0to5Inv.translation.y, T0to5Inv.translation.z ); // atan2(T0to5Inv(2,4),T0to5Inv(3,4));
	t4 = atan2( -T0to5Inv.translation.x, sqrt(T0to5Inv.translation.y*T0to5Inv.translation.y + T0to5Inv.translation.z*T0to5Inv.translation.z) ) - angle1;


	//    T0to5 = T0to2 * T2to3 * T3to4 * T4to5
	// => T0to2 = T0to5 * T4to5^-1 * T3to4^-1 * T2to3^-1

	RotationMatrix dummy; // somehow the static call doesn't work; has to be investigated
	Pose3D T4to5Inv(dummy.fromRotationX(-t5));
	Pose3D T3to4Inv(dummy.fromRotationY(-t4));
	Pose3D T2to3Inv(dummy.fromRotationY(-t3));

	Pose3D T0to2 = ((T0to5 * T4to5Inv) * T3to4Inv) * T2to3Inv;


	double c0,s0,c1,s1,c2,s2;



	double x = 1 / sqrt(2.0);

	double X1 = T0to2.rotation[0][0]; // = T0to2(1,1);
	//double X2 = T0to2(1,2);
	double X3 = T0to2.rotation[2][0]; // = T0to2(1,3);
	double X4 = T0to2.rotation[0][1]; // = T0to2(2,1);
	double X5 = T0to2.rotation[1][1]; // = T0to2(2,2);
	double X6 = T0to2.rotation[2][1]; // = T0to2(2,3);
	double X7 = T0to2.rotation[0][2]; // = T0to2(3,1);
	double X8 = T0to2.rotation[1][2]; // = T0to2(3,2);
	double X9 = T0to2.rotation[2][2]; // = T0to2(3,3);

	double X10 = sign * X6 + X9; // = c2(c1-s1)
	double X11 = sign * X4 + X7; // = s2(s1-c1)
	double X12 = X5 - sign * X8; // = c0(c1-s1)

	// X10 might be zero in the following cases:
	// 1.: c2 might be 0. This cannot be, because t2 is in [-100,25] and therefore can neither be -180 nor 180.
	// 2.: (c1-s1) might be 0. This can happen if t1 is 45 (or -135, but this is not possible).
	if (X10 == 0)
	{
		// ERROR HANDLING NEEDED!
		OUTPUT(idText,text,"something wrong in NaoKinematic");
    t0 = t1 = t2 = 0;
	}
	else // X10 != 0   => c2 != 0  && (c1-s1) != 0
	{
		t2 = atan( - X11 / X10 );
		s2 = sin(t2);
		c2 = cos(t2);

		c0 = c2 * X1 + s2 * X3;
		s0 = -(c2 * X7 + s2 * X9) / x;
		t0 = atan2(s0,c0);

		if (c0 == 0)
		{
			c1 = X5 + X10/(2*c2); // c1 != 0, see above
			s1 = X5 - X10/(2*c2);
		}
		else if (s2 == 0) // && c0 != 0)
		{
			double X14 = X5 - sign * X6; // = s1 + c0c1
			s1 = (X14 - X12) / (1 + c0); // c0 won't be -1, because t0 can neither be -180 nor 180
			c1 = (X14 + X12/c0) / (1 + c0);
		}
		else // (c0 != 0) && (s0 != 0)
		{
			double X13 = - X7/s2 - sign* X6/c2 + x*s0*s2/c2 - x*s0*c2/s2; // = c0(c1+s1)
			c1 = (X12 + X13)/(2*c0); // c0 != 0, because "else"...
			s1 = (-X12 + X13)/(2*c0);
		}
		t1 = atan2(s1,c1);
	}





	// sign's are necessary because bredo-framework uses a different joint calibration than the rotation directions shown in the above mentioned figure provided by aldebaran.
	jointRequest.angles[whichSideJoint0 + 0] = t0; // [-90 ;  0 ] = [...
	jointRequest.angles[whichSideJoint0 + 1] = t1; // [-100; 25 ] = [...
	jointRequest.angles[whichSideJoint0 + 2] = t2; // [-45 ; 25 ]
	jointRequest.angles[whichSideJoint0 + 3] = t3; // [ 0  ; 130]
	jointRequest.angles[whichSideJoint0 + 4] = t4; // [-75 ; 45 ]
	jointRequest.angles[whichSideJoint0 + 5] = sign * t5; // [-25 ; 45 ]
}


void NaoKinematic::update(KinematicOutput& kinematicOutput)
{
#ifndef WALKING_SIMULATOR
	MODIFY("NaoKinematic:rcxpKinematicRequest", rcxpKinematic);

	rcxpKinematic.calculated = false;
	DEBUG_RESPONSE("NaoKinematic:rcxpKinematic", 	
		Vector3<double> leftFootRCXP(rcxpKinematic.kinematicRequest[0], rcxpKinematic.kinematicRequest[1] , rcxpKinematic.kinematicRequest[2]);
	Vector3<double> leftRotRCXP(rcxpKinematic.kinematicRequest[3], rcxpKinematic.kinematicRequest[4] , rcxpKinematic.kinematicRequest[5]);
	Vector3<double> rightFootRCXP(rcxpKinematic.kinematicRequest[6], rcxpKinematic.kinematicRequest[7], rcxpKinematic.kinematicRequest[8]);
	Vector3<double> rightRotRCXP(rcxpKinematic.kinematicRequest[9], rcxpKinematic.kinematicRequest[10], rcxpKinematic.kinematicRequest[11]);
	calcLegJoints(JointData::legLeft0, leftFootRCXP, leftRotRCXP, rcxpKinematic.jointRequest);
	calcLegJoints(JointData::legRight0, rightFootRCXP, rightRotRCXP, rcxpKinematic.jointRequest);

	rcxpKinematic.calculated = true;
	);
#endif

	Vector3<double> leftFoot(theKinematicRequest.leftFoot[0], theKinematicRequest.leftFoot[1], theKinematicRequest.leftFoot[2]);
	Vector3<double> rightFoot(theKinematicRequest.rightFoot[0], theKinematicRequest.rightFoot[1], theKinematicRequest.rightFoot[2]);
	Vector3<double> leftFootRot(theKinematicRequest.leftFoot[3], theKinematicRequest.leftFoot[4], theKinematicRequest.leftFoot[5]);
	Vector3<double> rightFootRot(theKinematicRequest.rightFoot[3], theKinematicRequest.rightFoot[4], theKinematicRequest.rightFoot[5]);

	//double mean=0;

	double distLeft=sqrt(sqr(leftFoot[0])+sqr(leftFoot[1])+sqr(leftFoot[2]));
	double distRight=sqrt(sqr(rightFoot[0])+sqr(rightFoot[1])+sqr(rightFoot[2]));

	int baseFoot;

	switch (theKinematicRequest.kinematicType) 
	{

	case KinematicRequest::feet :
		// Stand on the foot, which is nearer to the center
		// This is foot has more pressure when the robot moves slowly
		if (distLeft<distRight)
		{
			baseFoot=LEFT_FOOT;
			calcLegJoints(JointData::legLeft0, leftFoot, leftFootRot, kinematicOutput);
			calcLegJoints(JointData::legRight0, rightFoot, rightFootRot, kinematicOutput.angles[JointData::legLeft0], kinematicOutput);
		}
		else
		{
			baseFoot=RIGHT_FOOT;
			calcLegJoints(JointData::legRight0, rightFoot, rightFootRot, kinematicOutput);
			calcLegJoints(JointData::legLeft0, leftFoot, leftFootRot, kinematicOutput.angles[JointData::legRight0], kinematicOutput);

		}
		break;

	case KinematicRequest::bodyAndLeftFoot :
		baseFoot=LEFT_FOOT;
		rightFoot=checkConstraints(leftFoot, leftFootRot.z, rightFoot, rightFootRot.z, false);
		calcLegJoints(JointData::legLeft0, leftFoot, leftFootRot, kinematicOutput);
		calcLegJoints(JointData::legRight0, rightFoot, rightFootRot, kinematicOutput.angles[JointData::legLeft0], kinematicOutput);
		//kinematicOutput.angles[JointData::legLeft1]+=0.1;
		break;

	case KinematicRequest::bodyAndRightFoot :	  
		baseFoot=RIGHT_FOOT;
		leftFoot=checkConstraints(leftFoot, leftFootRot.z, rightFoot, rightFootRot.z, true);
		calcLegJoints(JointData::legRight0, rightFoot, rightFootRot, kinematicOutput);
		calcLegJoints(JointData::legLeft0, leftFoot, leftFootRot, kinematicOutput.angles[JointData::legRight0], kinematicOutput);
		//kinematicOutput.angles[JointData::legRight1]-=0.1;
		break;
	default:
		break;

	}
#ifdef WALKING_SIMULATOR
	for (int i=(int)JointData::legLeft0; i<12; i++)
	{
		kinematicOutput.angles[i]+=theKinematicRequest.offsets.angles[i];
	}
#endif
	kinematicOutput.angles[JointData::armLeft0] = JointData::ignore;
	kinematicOutput.angles[JointData::armLeft1] = JointData::ignore;
	kinematicOutput.angles[JointData::armLeft2] = JointData::ignore;
	kinematicOutput.angles[JointData::armLeft3] = JointData::ignore;

	kinematicOutput.angles[JointData::armRight0] = JointData::ignore;
	kinematicOutput.angles[JointData::armRight1] = JointData::ignore;
	kinematicOutput.angles[JointData::armRight2] = JointData::ignore;
	kinematicOutput.angles[JointData::armRight3] = JointData::ignore;
	kinematicOutput.angles[JointData::headTilt] = JointData::ignore;
	kinematicOutput.angles[JointData::headPan] = JointData::ignore;

	if ((kinematicOutput.angles[JointData::legLeft0] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legLeft0] == JointData::off))
		kinematicOutput.angles[JointData::legLeft0] = theKinematicRequest.offsets.angles[JointData::legLeft0];
	else
		kinematicOutput.angles[JointData::legLeft0] += theKinematicRequest.offsets.angles[JointData::legLeft0];

	if ((kinematicOutput.angles[JointData::legLeft1] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legLeft1] == JointData::off))
		kinematicOutput.angles[JointData::legLeft1] = theKinematicRequest.offsets.angles[JointData::legLeft1];
	else
		kinematicOutput.angles[JointData::legLeft1] += theKinematicRequest.offsets.angles[JointData::legLeft1];

	if ((kinematicOutput.angles[JointData::legLeft2] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legLeft2] == JointData::off))
		kinematicOutput.angles[JointData::legLeft2] = theKinematicRequest.offsets.angles[JointData::legLeft2];
	else
		kinematicOutput.angles[JointData::legLeft2] += theKinematicRequest.offsets.angles[JointData::legLeft2];

	if ((kinematicOutput.angles[JointData::legLeft3] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legLeft3] == JointData::off))
		kinematicOutput.angles[JointData::legLeft3] = theKinematicRequest.offsets.angles[JointData::legLeft3];
	else
		kinematicOutput.angles[JointData::legLeft3] += theKinematicRequest.offsets.angles[JointData::legLeft3];

	if ((kinematicOutput.angles[JointData::legLeft4] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legLeft4] == JointData::off))
		kinematicOutput.angles[JointData::legLeft4] = theKinematicRequest.offsets.angles[JointData::legLeft4];
	else
		kinematicOutput.angles[JointData::legLeft4] += theKinematicRequest.offsets.angles[JointData::legLeft4];

	if ((kinematicOutput.angles[JointData::legLeft5] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legLeft5] == JointData::off))
		kinematicOutput.angles[JointData::legLeft5] = theKinematicRequest.offsets.angles[JointData::legLeft5];
	else
		kinematicOutput.angles[JointData::legLeft5] += theKinematicRequest.offsets.angles[JointData::legLeft5];


	if ((kinematicOutput.angles[JointData::legRight0] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legRight0] == JointData::off))
		kinematicOutput.angles[JointData::legRight0] = theKinematicRequest.offsets.angles[JointData::legRight0];
	else
		kinematicOutput.angles[JointData::legRight0] += theKinematicRequest.offsets.angles[JointData::legRight0];

	if ((kinematicOutput.angles[JointData::legRight1] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legRight1] == JointData::off))
		kinematicOutput.angles[JointData::legRight1] = theKinematicRequest.offsets.angles[JointData::legRight1];
	else
		kinematicOutput.angles[JointData::legRight1] += theKinematicRequest.offsets.angles[JointData::legRight1];

	if ((kinematicOutput.angles[JointData::legRight2] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legRight2] == JointData::off))
		kinematicOutput.angles[JointData::legRight2] = theKinematicRequest.offsets.angles[JointData::legRight2];
	else
		kinematicOutput.angles[JointData::legRight2] += theKinematicRequest.offsets.angles[JointData::legRight2];

	if ((kinematicOutput.angles[JointData::legRight3] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legRight3] == JointData::off))
		kinematicOutput.angles[JointData::legRight3] = theKinematicRequest.offsets.angles[JointData::legRight3];
	else
		kinematicOutput.angles[JointData::legRight3] += theKinematicRequest.offsets.angles[JointData::legRight3];

	if ((kinematicOutput.angles[JointData::legRight4] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legRight4] == JointData::off))
		kinematicOutput.angles[JointData::legRight4] = theKinematicRequest.offsets.angles[JointData::legRight4];
	else
		kinematicOutput.angles[JointData::legRight4] += theKinematicRequest.offsets.angles[JointData::legRight4];

	if ((kinematicOutput.angles[JointData::legRight5] == JointData::ignore)
		||(kinematicOutput.angles[JointData::legRight5] == JointData::off))
		kinematicOutput.angles[JointData::legRight5] = theKinematicRequest.offsets.angles[JointData::legRight5];
	else
		kinematicOutput.angles[JointData::legRight5] += theKinematicRequest.offsets.angles[JointData::legRight5];

#ifdef WARN_ANGLES
	if (kinematicOutput.angles[JointData::legLeft0]<-1.58 || kinematicOutput.angles[JointData::legLeft0]>0.01)
		OUTPUT(idText, text, "Angle out of Range (legLeft0): " << kinematicOutput.angles[JointData::legLeft0]);
	if (kinematicOutput.angles[JointData::legLeft1]<-0.79 || kinematicOutput.angles[JointData::legLeft1]>0.44)
		OUTPUT(idText, text, "Angle out of Range (legLeft1): " << kinematicOutput.angles[JointData::legLeft1]);
	if (kinematicOutput.angles[JointData::legLeft2]<-1.75 || kinematicOutput.angles[JointData::legLeft2]>0.44)
		OUTPUT(idText, text, "Angle out of Range (legLeft2): " << kinematicOutput.angles[JointData::legLeft2]);
	if (kinematicOutput.angles[JointData::legLeft3]<-0.01 || kinematicOutput.angles[JointData::legLeft3]>2.27)
		OUTPUT(idText, text, "Angle out of Range (legLeft3): " << kinematicOutput.angles[JointData::legLeft3]);
	if (kinematicOutput.angles[JointData::legLeft4]<-1.31 || kinematicOutput.angles[JointData::legLeft4]>0.79)
		OUTPUT(idText, text, "Angle out of Range (legLeft4): " << kinematicOutput.angles[JointData::legLeft4]);
	if (kinematicOutput.angles[JointData::legLeft5]<-0.44 || kinematicOutput.angles[JointData::legLeft5]>0.79)
		OUTPUT(idText, text, "Angle out of Range (legLeft5): " << kinematicOutput.angles[JointData::legLeft5]);
	if (kinematicOutput.angles[JointData::legRight0]<-1.58 || kinematicOutput.angles[JointData::legRight0]>0.01)
		OUTPUT(idText, text, "Angle out of Range (legRight0): " << kinematicOutput.angles[JointData::legRight0]);
	if (kinematicOutput.angles[JointData::legRight1]<-0.79 || kinematicOutput.angles[JointData::legRight1]>0.44)
		OUTPUT(idText, text, "Angle out of Range (legRight1): " << kinematicOutput.angles[JointData::legRight1]);
	if (kinematicOutput.angles[JointData::legRight2]<-1.75 || kinematicOutput.angles[JointData::legRight2]>0.44)
		OUTPUT(idText, text, "Angle out of Range (legRight2): " << kinematicOutput.angles[JointData::legRight2]);
	if (kinematicOutput.angles[JointData::legRight3]<-0.01 || kinematicOutput.angles[JointData::legRight3]>2.27)
		OUTPUT(idText, text, "Angle out of Range (legRight3): " << kinematicOutput.angles[JointData::legRight3]);
	if (kinematicOutput.angles[JointData::legRight4]<-1.31 || kinematicOutput.angles[JointData::legRight4]>0.79)
		OUTPUT(idText, text, "Angle out of Range (legRight4): " << kinematicOutput.angles[JointData::legRight4]);
	if (kinematicOutput.angles[JointData::legRight5]<-0.44 || kinematicOutput.angles[JointData::legRight5]>0.79)
		OUTPUT(idText, text, "Angle out of Range (legRight5): " << kinematicOutput.angles[JointData::legRight5]);
#endif
	LOG("ExternalSimulator.csv", "LHipRoll", kinematicOutput.angles[JointData::legLeft1]);
	LOG("ExternalSimulator.csv", "LHipPitch", kinematicOutput.angles[JointData::legLeft2]);
	LOG("ExternalSimulator.csv", "LKneePitch", kinematicOutput.angles[JointData::legLeft3]);
	LOG("ExternalSimulator.csv", "LAnklePitch", kinematicOutput.angles[JointData::legLeft4]);
	LOG("ExternalSimulator.csv", "LAnkleRoll", kinematicOutput.angles[JointData::legLeft5]);

	LOG("ExternalSimulator.csv", "RHipRoll", kinematicOutput.angles[JointData::legRight1]);
	LOG("ExternalSimulator.csv", "RHipPitch", kinematicOutput.angles[JointData::legRight2]);
	LOG("ExternalSimulator.csv", "RKneePitch", kinematicOutput.angles[JointData::legRight3]);
	LOG("ExternalSimulator.csv", "RAnklePitch", kinematicOutput.angles[JointData::legRight4]);
	LOG("ExternalSimulator.csv", "RAnkleRoll", kinematicOutput.angles[JointData::legRight5]); 
}

MAKE_MODULE(NaoKinematic, Dortmund WalkingEngine)
