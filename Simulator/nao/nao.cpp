/*
	Copyright 2011, Oliver Urbann
	All rights reserved.

	This file is part of MoToFlex.

    MoToFlex is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MoToFlex is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

	Contact e-mail: oliver.urbann@tu-dortmund.de
*/
#include "nao.h"
#include "genom.h"

#include "csvlogger.h" 
#include "math/homMatrix.h"
#include "math/Common.h"

#ifdef CART_MODEL
#define BODY_W		4.346f
#define BODY_P		0.046f
#define UPPER_LEG_W	0.001f
#define LOWER_LEG_W	0.001f
#define FOOT_W		0.001f
#define ARM_W		0.001f
#define HEAD_W		0.001f	
#else
#ifdef AKKU
#define BODY_W		1.217f+0.335f
#else
#define BODY_W		1.217f
#endif
#define BODY_P		0.046466f
#define UPPER_LEG_W	0.533f
#define LOWER_LEG_W	0.423f
#define FOOT_W		0.158f
#define ARM_W		0.25f
#define HEAD_W		0.401f
#endif

#ifndef ARMS
#undef BODY_W
#undef BODY_P
#define BODY_P		0.0618f
#define BODY_W		2.118f
#endif

extern Genom parms;

Nao::Nao() : created(false) 
{
	ankleToGround=Vector3<float>(0, 0, -0.046f);
	kneeToAnkle=Vector3<float>(0, 0, -0.1f);
	hipToKnee=Vector3<float>(0, 0, -0.1f);
	bodyToRightHip=Vector3<float>(0, -0.05f, -0.085f);
}

void Nao::stop()
{
	created = false;
	for (int i = 0; i < numOfBoxes; i++)
		boxes[i].destroyPhysics();
	for (int i = 0; i < numOfLegs; i++)
	{
		hip[i].destroyPhysics();
		ankle[i].destroyPhysics();
		knee[i].destroyPhysics();
	}
#ifdef MOVEABLE_ARMS
	for (int i = 0; i < numOfArms; i++)
		shoulder[i].destroyPhysics();
#endif
}

int Nao::getNumOfJoints()
{
	return NUM_OF_JOINTS;
}

void Nao::create(dWorldID world, dSpaceID space, ActionType actionType, string name)
{
	float stiffness=(float)parms.stiffness;
	if (actionType==specialAction)
		stiffness=1.0;
	this->name=name;
	const double armAngle=0.35;
	dReal flexaxis[]={0, 0, 1};
#pragma region Body
	// position is the root of the coordinate system over
	// the ground plus position of the center of mass
	dReal bodyPos[]={-0.007829f, 0, 0.331f+BODY_P};
	dReal bodySize[]={0.10f, 0.11f, 0.108f};
	boxes[body].createPhysics(world, space, bodyPos, bodySize, BODY_W);
#pragma endregion
	
	// Height of the foot, plus height of the lower leg,
	// plus the z pos of the CoM of the upper leg
	dReal upperLegPos[]={-0.005579f, 0, 0.046f+0.100f+0.062f};

	// same here
	dReal lowerLegPos[]={-0.000511f, 0, 0.046f+0.0309f};
	dReal footPos[]={0.025f, 0, 0.016f};

	dReal upperLegSize[]={0.05f, 0.05f, 0.076f};
	dReal lowerLegSize[]={0.05f, 0.05f, 0.0618f};

	// This size is not for a correct CoM, but for a 
	// 100% correct foot size of the real nao. This is more
	// important, since the foot touches the ground at other
	// points with a smaller foot
	dReal footSize[]={0.157f, 0.087f, 0.032f};											

	dReal headSize[]={0.08f, 0.13f, 0.087f};
	dReal headPos[]={-0.000247f, 0, (float)(0.331+0.170899)};

	dReal armSize[]={0.045f, 0.045f, 2*0.0966142f};
#ifdef MOVEABLE_ARMS
    dReal armPos[]={0, 0.098, 0.331+0.1-armSize[2]/2};
#else
	dReal armPos[]={0, (float)(0.098+sin(armAngle)*0.0966142), (float)(0.331+0.1-cos(armAngle)*0.0966142)};
#endif
	
#pragma region leftLeg
	upperLegPos[1]+=0.05f;
	lowerLegPos[1]+=0.05f;
	footPos[1]+=0.05f;

	flexboxes[upperLegLeft].createPhysics(world, space, upperLegPos, upperLegSize, flexaxis, UPPER_LEG_W, parms.numOfSubBoxes);
	flexboxes[lowerLegLeft].createPhysics(world, space, lowerLegPos, lowerLegSize, flexaxis, LOWER_LEG_W, parms.numOfSubBoxes);
	boxes[footLeft].createPhysics(world, space, footPos, footSize, FOOT_W);
#pragma endregion

#pragma region rightLeg
	upperLegPos[1]-=0.1f;
	lowerLegPos[1]-=0.1f;
	footPos[1]-=0.1f;

	flexboxes[upperLegRight].createPhysics(world, space, upperLegPos, upperLegSize, flexaxis, UPPER_LEG_W, parms.numOfSubBoxes);
	flexboxes[lowerLegRight].createPhysics(world, space, lowerLegPos, lowerLegSize, flexaxis, LOWER_LEG_W, parms.numOfSubBoxes);
	boxes[footRight].createPhysics(world, space, footPos, footSize, FOOT_W);
#pragma endregion

#pragma region arms
#ifdef ARMS
	boxes[armLeft].createPhysics(world, space, armPos, armSize, ARM_W);
	armPos[1]=-armPos[1];
	boxes[armRight].createPhysics(world, space, armPos, armSize, ARM_W);
	boxes[head].createPhysics(world, space, headPos, headSize, HEAD_W);
#endif
#pragma endregion
	dReal hipAnchor[]={0, 0.05f, (float)(0.046+0.1+0.1)};
	dReal kneeAnchor[]={0, 0.05f, (float)(0.046+0.1)};
	dReal footAnchor[]={0, 0.05f, 0.046f};


#pragma region Joints
	dReal xAxis[]={1, 0, 0};
	dReal yAxis[]={0, 1, 0};
	dReal zAxis[]={0, 0, 1};

	hip[leftLeg].createPhysics(world,
		boxes[body],
		*flexboxes[upperLegLeft].getBox(parms.numOfSubBoxes-1),
		hipAnchor,
		xAxis,		// Reihenfolge der Achsen durch die Reihenfolge
		yAxis,		// der K�rperverbindungen vorgegeben
		stiffness,
		0,
		1);

	hip[leftLeg].setName("LeftHip");
	
	knee[leftLeg].createPhysics(world,
		*flexboxes[upperLegLeft].getBox(0),
		*flexboxes[lowerLegLeft].getBox(parms.numOfSubBoxes-1),
		kneeAnchor,
		yAxis,
		1,
		stiffness);

	knee[leftLeg].setName("LeftKnee");

	ankle[leftLeg].createPhysics(world,
		*flexboxes[lowerLegLeft].getBox(0),
		boxes[footLeft],
		footAnchor,
		yAxis,
		xAxis,
		stiffness,
		1,
#if NUM_OF_GEARTYPES==3
		0);
#else
		0);
#endif

	ankle[leftLeg].setName("LeftAnkle");

	hipAnchor[1]-=0.1f;
	kneeAnchor[1]-=0.1f;
	footAnchor[1]-=0.1f;

	hip[rightLeg].createPhysics(world,
		boxes[body],
		*flexboxes[upperLegRight].getBox(parms.numOfSubBoxes-1),
		hipAnchor,
		xAxis,		// Reihenfolge der Achsen durch die Reihenfolge
		yAxis,		// der K�rperverbindungen vorgegeben
		stiffness,
		0,
		1);

	hip[rightLeg].setName("RightHip");

	knee[rightLeg].createPhysics(world,
		*flexboxes[upperLegRight].getBox(0),
		*flexboxes[lowerLegRight].getBox(parms.numOfSubBoxes-1),
		kneeAnchor,
		yAxis,
		1,
		stiffness);

	knee[rightLeg].setName("RightKnee");
	
	ankle[rightLeg].createPhysics(world,
		*flexboxes[lowerLegRight].getBox(0),
		boxes[footRight],
		footAnchor,
		yAxis,
		xAxis,
		stiffness,
		1,
#if NUM_OF_GEARTYPES==3
		0);
#else
		0);
#endif
	ankle[rightLeg].setName("RightAnkle");
	
#ifdef ARMS
	dJointID id;

#ifndef MOVEABLE_ARMS
	
	dMatrix3 R;
	dRFromAxisAndAngle(R, 1, 0, 0, (float)-armAngle);
	dBodySetRotation(boxes[armRight].getID(), R); 
	dRFromAxisAndAngle(R, 1, 0, 0, (float)armAngle);
	dBodySetRotation(boxes[armLeft].getID(), R);

	id=dJointCreateFixed(world, 0);
	dJointAttach(id, boxes[body].getID(), boxes[armRight].getID());
	dJointSetFixed(id);

	id=dJointCreateFixed(world, 0);
	dJointAttach(id, boxes[body].getID(), boxes[armLeft].getID());
	dJointSetFixed(id);
#else
	dReal armAnchor[3];
	armAnchor[0]=armPos[0];
	armAnchor[1]=-armPos[1];
	armAnchor[2]=armPos[2]+armSize[2]/2;
	shoulder[leftArm].createPhysics( world,
		boxes[body],
		boxes[armLeft],
		armAnchor,
		yAxis,
		xAxis);

	armAnchor[1]-=0.098*2;
	
	shoulder[rightArm].createPhysics( world,
		boxes[body],
		boxes[armRight],
		armAnchor,
		yAxis,
		xAxis);

#endif

	id=dJointCreateFixed(world, 0);
	dJointAttach(id, boxes[body].getID(), boxes[head].getID());
	dJointSetFixed(id);
#endif
#pragma endregion

	created=true;
}

void Nao::draw(double yOffset)
{
	for (int i=0; i<numOfBoxes; i++)
		boxes[i].draw(yOffset);
	for (int i=0; i<numOfFlexBoxes; i++)
		flexboxes[i].draw(yOffset);
}

dBodyID Nao::getBodyID(int boxNum)
{
	return boxes[boxNum].getID();
}

Joint *Nao::getJoint(int motorID)
{
	if (!created) return NULL;

	switch(motorID)
	{
	case lHipRoll:
	case lHipPitch:
		return &hip[leftLeg];
		break;

	case rHipRoll:
	case rHipPitch:
		return &hip[rightLeg];
		break;

	case lKneePitch:
		return &knee[leftLeg];
		break;

	case rKneePitch:
		return &knee[rightLeg];
		break;

	case lAnkleRoll:
	case lAnklePitch:
		return &ankle[leftLeg];
		break;

	case rAnklePitch:
	case rAnkleRoll:
		return &ankle[rightLeg];
		break;
	}

	return NULL;
}

void Nao::getBodyOrientationQuaternion(dReal *quat)
{
	if (!created) return;
	boxes[body].getBodyOrientationQuaternion(quat);
}

void Nao::getOrientation(int boxNum, dReal *orientation)
{
	if (!created) return;
	return boxes[boxNum].getOrientation(orientation);
}

void Nao::getOrientation(dReal *orientation)
{
	if (!created) return;
	return boxes[body].getOrientation(orientation);
}

RotationMatrix Nao::getOrientation()
{
	if (!created)
	{
		RotationMatrix empty;
		return empty;
	}
	return boxes[body].getRotationMatrix();
}

void Nao::getPosition(dReal *position)
{
	if (!created) return;
	boxes[body].getPosition(position);
}

void Nao::getVelocity(dReal *velocity)
{
	if (!created) return;
	boxes[body].getVelocity(velocity);
}

void Nao::getAngularVelocity(dReal *velocity)
{
	if (!created) return;
	boxes[body].getAngularVelocity(velocity);
}

void Nao::getPosition(int boxNum, dReal *position)
{
	if (!created) return;
	return boxes[boxNum].getPosition(position);
}

void Nao::addForce(int boxNum, dReal fx, dReal fy, dReal fz)
{
	if (!created) return;
	boxes[boxNum].addForce(fx, fy, fz);
}

Vector3<double> Nao::getLeftFootForce()
{
	dReal f[4];
	boxes[footLeft].getForce(f);
	return Vector3<double>(f[0], f[1], f[2]);
}

Vector3<double> Nao::getRightFootForce()
{
	dReal f[4];
	boxes[footRight].getForce(f);
	return Vector3<double>(f[0], f[1], f[2]);
}

Vector3<double> Nao::getPosition()
{
	dReal p[4];
	getPosition(p);
	return Vector3<double>(p[0], p[1], p[2]);
}

Vector3<double> Nao::getVelocity()
{
	dReal v[4];
	getVelocity(v);
	return Vector3<double>(v[0], v[1], v[2]);
}
	/**
	 * METHOD ADDED FOR PERIODIC REWARD COMPOSITION
	*/
Vector3<double> Nao::getLeftFootVelocity()
{
	dReal v[4];
	boxes[footLeft].getVelocity(v);
	return Vector3<double>(v[0], v[1], v[2]);
}

	/**
	 * METHOD ADDED FOR PERIODIC REWARD COMPOSITION
	*/
Vector3<double> Nao::getRightFootVelocity()
{
	dReal v[4];
	boxes[footRight].getVelocity(v);
	return Vector3<double>(v[0], v[1], v[2]);
}

void Nao::getSize(int boxNum, dReal *size)
{
	dReal *s = boxes[boxNum].getSize();
	for (int i = 0; i < 3; i++)
		size[i] = s[i];
}

void Nao::setAngles(float angles[])
{	
	hip[leftLeg].setAngle(angles[lHipRoll], 1);		// Reihenfolge der Achsen beim ODE Joint
	hip[leftLeg].setAngle(angles[lHipPitch], 2);	// anders herum als beim Nao
	knee[leftLeg].setAngle(angles[lKneePitch]);	  
	ankle[leftLeg].setAngle(angles[lAnklePitch], 1);
	ankle[leftLeg].setAngle(angles[lAnkleRoll], 2);

	hip[rightLeg].setAngle(angles[rHipRoll], 1);
	hip[rightLeg].setAngle(angles[rHipPitch], 2);
	knee[rightLeg].setAngle(angles[rKneePitch]);
	ankle[rightLeg].setAngle(angles[rAnklePitch], 1);
	ankle[rightLeg].setAngle(angles[rAnkleRoll], 2);

#ifdef MOVEABLE_ARMS
	shoulder[leftArm].setAngle(angles[leftArmRoll],1);
	shoulder[leftArm].setAngle(angles[leftArmPitch],2);
	shoulder[rightArm].setAngle(angles[rightArmRoll],1);
	shoulder[rightArm].setAngle(angles[rightArmPitch],2);
#endif

}

void Nao::getAngles(float angles[])
{
	angles[lHipRoll]=hip[leftLeg].getAngle(1);		
	angles[lHipPitch]=hip[leftLeg].getAngle(2);
	angles[lKneePitch]=knee[leftLeg].getAngle();	  
	angles[lAnklePitch]=ankle[leftLeg].getAngle(1);
	angles[lAnkleRoll]=ankle[leftLeg].getAngle(2);

	angles[rHipRoll]=hip[rightLeg].getAngle(1);
	angles[rHipPitch]=hip[rightLeg].getAngle(2);
	angles[rKneePitch]=knee[rightLeg].getAngle();
	angles[rAnklePitch]=ankle[rightLeg].getAngle(1);
	angles[rAnkleRoll]=ankle[rightLeg].getAngle(2);	
#ifdef MOVEABLE_ARMS
	angles[leftArm]=shoulder[leftArm].getAngle(1);
	angles[leftArm]=shoulder[leftArm].getAngle(2);
	angles[rightArm]=shoulder[rightArm].getAngle(1);
	angles[rightArm]=shoulder[rightArm].getAngle(2);
#endif
}

void Nao::getJointVelocities(float angles[])
{
	angles[lHipRoll]=hip[leftLeg].getVelocity(1);		
	angles[lHipPitch]=hip[leftLeg].getVelocity(2);
	angles[lKneePitch]=knee[leftLeg].getVelocity();	  
	angles[lAnklePitch]=ankle[leftLeg].getVelocity(1);
	angles[lAnkleRoll]=ankle[leftLeg].getVelocity(2);

	angles[rHipRoll]=hip[rightLeg].getVelocity(1);
	angles[rHipPitch]=hip[rightLeg].getVelocity(2);
	angles[rKneePitch]=knee[rightLeg].getVelocity();
	angles[rAnklePitch]=ankle[rightLeg].getVelocity(1);
	angles[rAnkleRoll]=ankle[rightLeg].getVelocity(2);	
#ifdef MOVEABLE_ARMS
	angles[leftArm]=shoulder[leftArm].getVelocity(1);
	angles[leftArm]=shoulder[leftArm].getVelocity(2);
	angles[rightArm]=shoulder[rightArm].getVelocity(1);
	angles[rightArm]=shoulder[rightArm].getVelocity(2);
#endif
}

void Nao::setPIDParams(PIDController::PIDParams params[], Joint::Method method)
{
	hip[leftLeg].setPIDParams(params[lHipRoll], 1, method);
	hip[leftLeg].setPIDParams(params[lHipPitch], 2, method);
	knee[leftLeg].setPIDParams(params[lKneePitch], method);
	ankle[leftLeg].setPIDParams(params[lAnklePitch], 1, method);
	ankle[leftLeg].setPIDParams(params[lAnkleRoll], 2, method);

	hip[rightLeg].setPIDParams(params[rHipRoll], 1, method);
	hip[rightLeg].setPIDParams(params[rHipPitch], 2, method);
	knee[rightLeg].setPIDParams(params[rKneePitch], method);
	ankle[rightLeg].setPIDParams(params[rAnklePitch], 1, method);
	ankle[rightLeg].setPIDParams(params[rAnkleRoll], 2, method);

#ifdef MOVEABLE_ARMS
	shoulder[leftArm].setPIDParams(params[leftArmRoll], 1, method);
	shoulder[leftArm].setPIDParams(params[leftArmPitch], 2, method);
	shoulder[rightArm].setPIDParams(params[rightArmRoll], 1, method);
	shoulder[rightArm].setPIDParams(params[rightArmPitch], 2, method);
#endif
}

void Nao::setAllPIDParams(float p, float i, float d, Joint::Method method)
{
	PIDController::PIDParams pidParams[Nao::numOfMotors];
	for (int i=0; i<NUM_OF_JOINTS; i++)
	{
		pidParams[i].p=(float)p;
		pidParams[i].i=(float)i;
		pidParams[i].d=(float)d;
		pidParams[i].frameLen=1.0f/float(parms.controllerFreq);
	}
	setPIDParams(pidParams, method);
}

void Nao::act(Joint::Method method)
{
	for (int i=0; i<2; i++)
	{
		hip[i].act(method);
		knee[i].act(method);
		ankle[i].act(method);
	}

	for (int i=0; i<numOfFlexBoxes; i++)
		flexboxes[i].act((float)parms.sc);

#ifdef MOVEABLE_ARMS
	shoulder[leftArm].act(Joint::velocity);
	shoulder[rightArm].act(Joint::velocity);
#endif
}

void Nao::log()
{
	LOG(name, "LHipRoll", hip[leftLeg].getAngle(1)*converter[0]);
	LOG(name, "LHipPitch", hip[leftLeg].getAngle(2)*converter[1]);
	LOG(name, "LKneePitch", knee[leftLeg].getAngle()*converter[2]);
	LOG(name, "LAnklePitch", ankle[leftLeg].getAngle(1)*converter[3]);
	LOG(name, "LAnkleRoll", ankle[leftLeg].getAngle(2)*converter[4]);
	LOG(name, "RHipRoll", hip[rightLeg].getAngle(1)*converter[5]);
	LOG(name, "RHipPitch", hip[rightLeg].getAngle(2)*converter[6]);
	LOG(name, "RKneePitch", knee[rightLeg].getAngle()*converter[7]);
	LOG(name, "RAnklePitch", ankle[rightLeg].getAngle(1)*converter[8]);
	LOG(name, "RAnkleRoll", ankle[rightLeg].getAngle(2)*converter[9]);
}

float Nao::getWeight()
{
	float mass=0;
	for (int i=0; i<numOfBoxes; i++)
		mass+=boxes[i].getWeight();
	for (int i=0; i<numOfFlexBoxes; i++)
		mass+=flexboxes[i].getWeight();
	return mass;
}

Vector3<double> Nao::getCoM()
{
	float angles[NUM_OF_JOINTS];
	double rightMass, leftMass;
	Vector3<double> LCoM, RCoM;

	if (!created) return Vector3<double>();

	getAngles(angles);

#pragma region First: CoM of left leg

	{
		double tiltAngleOfFirstJointAxis = - pi / 2 + pi / -4;

		double cx = cos(tiltAngleOfFirstJointAxis);
		double sx = sin(tiltAngleOfFirstJointAxis);

		HomMatrix M1(
			RotationMatrix(
				Vector3<double>(1, 0, 0),
				Vector3<double>(0, cx, sx),
				Vector3<double>(0, -sx, cx)));

		HomMatrix M2(
			RotationMatrix(
				Vector3<double>(1, 0, 0),
				Vector3<double>(0, cx, -sx),
				Vector3<double>(0, sx, cx)));

		HomMatrix LCoM0(Vector3<double>(-0.0056, 0, -0.038));
		HomMatrix LCoM1(Vector3<double>(-0.0005, 0, -0.0691));
		HomMatrix LCoM2(Vector3<double>(0.018, 0, -0.03));

		HomMatrix LR0(RotationMatrix(Vector3<double>(0, 0, 1), 0));
		HomMatrix LR1(RotationMatrix(Vector3<double>(1, 0, 0), -angles[0]));
		HomMatrix LR2(RotationMatrix(Vector3<double>(0, 1, 0), angles[1]));
		HomMatrix LR3(RotationMatrix(Vector3<double>(0, 1, 0), angles[2]));
		HomMatrix LR4(RotationMatrix(Vector3<double>(0, 1, 0), angles[3]));
		HomMatrix LR5(RotationMatrix(Vector3<double>(1, 0, 0), -angles[4]));

		HomMatrix LT0(Vector3<double>(bodyToRightHip[0], -bodyToRightHip[1], bodyToRightHip[2]));
		HomMatrix LT1(hipToKnee);
		HomMatrix LT2(kneeToAnkle);

		LCoM0 = LT0 * M1 * LR0 * M2 * LR1 * LR2 * LCoM0;
		LCoM1 = LT0 * M1 * LR0 * M2 * LR1 * LR2 * LT1 * LR3 * LCoM1;
		LCoM2 = LT0 * M1 * LR0 * M2 * LR1 * LR2 * LT1 * LR3 * LT2 * LR4 * LR5 *  LCoM2;

		leftMass=flexboxes[upperLegLeft].getWeight()+flexboxes[lowerLegLeft].getWeight()+boxes[footLeft].getWeight();
	
		LCoM =
			(LCoM0.getTranslation() * flexboxes[upperLegLeft].getWeight() +
			LCoM1.getTranslation() * flexboxes[lowerLegLeft].getWeight() +
			LCoM2.getTranslation() * boxes[footLeft].getWeight()) / leftMass;
	}
#pragma endregion

#pragma region First: CoM of right leg
	{
		double tiltAngleOfFirstJointAxis = - pi / 2 + pi / 4;

		double cx = cos(tiltAngleOfFirstJointAxis);
		double sx = sin(tiltAngleOfFirstJointAxis);

		HomMatrix M1(
			RotationMatrix(
				Vector3<double>(1, 0, 0),
				Vector3<double>(0, cx, sx),
				Vector3<double>(0, -sx, cx)));

		HomMatrix M2(
			RotationMatrix(
				Vector3<double>(1, 0, 0),
				Vector3<double>(0, cx, -sx),
				Vector3<double>(0, sx, cx)));

		HomMatrix RCoM0(Vector3<double>(-0.0056, 0, -0.038));
		HomMatrix RCoM1(Vector3<double>(-0.0005, 0, -0.0691));
		HomMatrix RCoM2(Vector3<double>(0.018, 0, -0.03));

		HomMatrix RR0(RotationMatrix(Vector3<double>(0, 0, 1), 0));
		HomMatrix RR1(RotationMatrix(Vector3<double>(1, 0, 0), angles[5]));
		HomMatrix RR2(RotationMatrix(Vector3<double>(0, 1, 0), angles[6]));
		HomMatrix RR3(RotationMatrix(Vector3<double>(0, 1, 0), angles[7]));
		HomMatrix RR4(RotationMatrix(Vector3<double>(0, 1, 0), angles[8]));
		HomMatrix RR5(RotationMatrix(Vector3<double>(1, 0, 0), angles[9]));

		HomMatrix RT0(bodyToRightHip);
		HomMatrix RT1(hipToKnee);
		HomMatrix RT2(kneeToAnkle);

		RCoM0 = RT0 * M1 * RR0 * M2 * RR1 * RR2 * RCoM0;
		RCoM1 = RT0 * M1 * RR0 * M2 * RR1 * RR2 * RT1 * RR3 * RCoM1;
		RCoM2 = RT0 * M1 * RR0 * M2 * RR1 * RR2 * RT1 * RR3 * RT2 * RR4 * RR5 *  RCoM2;

		rightMass=flexboxes[upperLegRight].getWeight()+flexboxes[lowerLegRight].getWeight()+boxes[footRight].getWeight();

		RCoM =
			(RCoM0.getTranslation() * flexboxes[upperLegRight].getWeight() +
			RCoM1.getTranslation() * flexboxes[lowerLegRight].getWeight() +
			RCoM2.getTranslation() * boxes[footRight].getWeight()) / rightMass;
	}
#pragma endregion

#ifdef MOVEABLE_ARMS
	
#else
	double bodyMass=2.1180;
#endif

	double massRobot=bodyMass+rightMass+leftMass;
	Vector3<double> CoM=
		(Vector3<double>(-0.0045, 0, 0.0612)*bodyMass +
		RCoM*rightMass + LCoM*leftMass)/massRobot;

	return CoM;
}

Vector3<double> Nao::getAcc()
{
	return Vector3<double>();
}

Vector3<double> Nao::getGyr()
{
	return Vector3<double>();
}
