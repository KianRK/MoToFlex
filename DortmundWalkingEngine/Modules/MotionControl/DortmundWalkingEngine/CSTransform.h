#pragma once
#include "Tools/Math/homMatrix.h"
#include "Tools/Math/Matrix_mxn.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Common.h"

class CSTransform
{
public:
	 CSTransform() : 
		tiltAngleOfFirstJointAxis(- pi / 2 + pi / 4),
		cx(cos(tiltAngleOfFirstJointAxis)),
		sx(sin(tiltAngleOfFirstJointAxis)),
		M1(
			RotationMatrix(
				Vector3<double>(1, 0, 0),
				Vector3<double>(0, cx, sx),
				Vector3<double>(0, -sx, cx))),
		M2(
			RotationMatrix(
				Vector3<double>(1, 0, 0),
				Vector3<double>(0, cx, -sx),
				Vector3<double>(0, sx, cx))),
		ankleToGroundTrans(Vector3<double>(0, 0, 0.046f)),
		kneeToAnkleTrans(Vector3<double>(0, 0, 0.101f)),
		hipToKneeTrans(Vector3<double>(0, 0, 0.101f)),
		bodyToRightHipTrans(Vector3<double>(0, 0.05f, 0.085f)),
		bodyToLeftHipTrans(Vector3<double>(0, -0.05f, 0.085f))
	  {};

	HomMatrix fromBodyToUpperRightLeg(double hipYawPitch, double hipRoll, double hipPitch);
	HomMatrix fromBodyToUpperLeftLeg(double hipYawPitch, double hipRoll, double hipPitch);
	HomMatrix fromUpperLegToLowerLeg(double kneePitch);
	HomMatrix fromLowerLegToFoot(double anklePitch, double ankleRoll);
	HomMatrix fromBodyToLeftGround(double hipYawPitch, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll);
	HomMatrix fromBodyToRightGround(double hipYawPitch, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll);

private:
	double tiltAngleOfFirstJointAxis, 
		cx, 
		sx;

	HomMatrix M1, 
		M2, 
		ankleToGroundTrans, 
		kneeToAnkleTrans, 
		hipToKneeTrans, 
		bodyToRightHipTrans, 
		bodyToLeftHipTrans;
};
