#include "CSTransform.h"

HomMatrix CSTransform::fromBodyToUpperRightLeg(double hipYawPitch, double hipRoll, double hipPitch)
{
	HomMatrix R0(RotationMatrix(Vector3<double>(0, 0, 1), -hipYawPitch));
	HomMatrix R1(RotationMatrix(Vector3<double>(1, 0, 0), -hipRoll));
	HomMatrix R2(RotationMatrix(Vector3<double>(0, 1, 0), -hipPitch));

	//return bodyToRightHipTrans * M1 * R0 * M2 * R1 * R2;
	return R2 * R1 * M2 * R0 * M1 * bodyToRightHipTrans;
}


HomMatrix CSTransform::fromBodyToUpperLeftLeg(double hipYawPitch, double hipRoll, double hipPitch)
{
	HomMatrix R0(RotationMatrix(Vector3<double>(0, 0, 1), -hipYawPitch));
	HomMatrix R1(RotationMatrix(Vector3<double>(1, 0, 0), -hipRoll));
	HomMatrix R2(RotationMatrix(Vector3<double>(0, 1, 0), -hipPitch));

	return R2 * R1 * M2 * R0 * M1 * bodyToLeftHipTrans;
}

HomMatrix CSTransform::fromUpperLegToLowerLeg(double kneePitch)
{
	HomMatrix R3(RotationMatrix(RotationMatrix(Vector3<double>(0, 1, 0), -kneePitch)));

	//return hipToKneeTrans * R3;
	return R3 * hipToKneeTrans;
}

HomMatrix CSTransform::fromLowerLegToFoot(double anklePitch, double ankleRoll)
{
	HomMatrix R4(RotationMatrix(Vector3<double>(0, 1, 0), -anklePitch));
	HomMatrix R5(RotationMatrix(Vector3<double>(1, 0, 0), -ankleRoll));

	//return kneeToAnkleTrans * R4 * R5;
	return R5 * R4 * kneeToAnkleTrans;
}

HomMatrix CSTransform::fromBodyToLeftGround(double hipYawPitch, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll)
{
	return ankleToGroundTrans * 
		fromLowerLegToFoot(anklePitch, ankleRoll) *
		fromUpperLegToLowerLeg(kneePitch) *
		fromBodyToUpperLeftLeg(hipYawPitch, hipRoll, hipPitch);
}

HomMatrix CSTransform::fromBodyToRightGround(double hipYawPitch, double hipRoll, double hipPitch, double kneePitch, double anklePitch, double ankleRoll)
{
	return ankleToGroundTrans *
		fromLowerLegToFoot(anklePitch, ankleRoll) *
		fromUpperLegToLowerLeg(kneePitch) *
		fromBodyToUpperRightLeg(hipYawPitch, hipRoll, hipPitch);
		
}
