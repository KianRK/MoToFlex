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
#include "doublejoint.h"
#include <genom.h>
#include "eaconfig.h"
#include <csvlogger.h>

#ifdef TRACING
#include <iostream>
using namespace std;
#endif


extern Genom parms;

void DoubleJoint::createPhysics(dWorldID world,
								Box &body1,
								Box &body2,
								dReal anchor[],
								dReal axis1[],
								dReal axis2[],
								float stiffness,
								short reductionType1,
								short reductionType2)
{
#ifdef TRACING
	cout << "createPhysics()" << endl;
	cout.flush();
#endif

	motor[0].init();
	motor[1].init();
	jointID=dJointCreateUniversal(world, 0);
	dJointAttach(jointID, body1.getID(), body2.getID());
	dJointSetUniversalAnchor(jointID, anchor[0], anchor[1], anchor[2]);
	dJointSetUniversalAxis1(jointID, axis1[0], axis1[1], axis1[2]);
	dJointSetUniversalAxis2(jointID, axis2[0], axis2[1], axis2[2]);

	motorID=dJointCreateAMotor(world, 0);
	dJointAttach(motorID,  body1.getID(), body2.getID());
	dJointSetAMotorMode(motorID, dAMotorUser);
	dJointSetAMotorNumAxes(motorID, 2);
	dJointSetAMotorAxis(motorID, 0, 1, axis1[0], axis1[1], axis1[2]);
	dJointSetAMotorAxis(motorID, 1, 1, axis2[0], axis2[1], axis2[2]);
	dJointSetFeedback(motorID, &fb);

	motor[0].setReductionType(reductionType1);
	motor[0].stiffness=stiffness;
	motor[1].setReductionType(reductionType2);
	motor[1].stiffness=stiffness;
}

void DoubleJoint::destroyPhysics()
{
	dJointDestroy(jointID);
	dJointDestroy(motorID);
}

string DoubleJoint::getName(short axis)
{
#ifdef TRACING
	cout << "getName(" <<  axis << ")" << endl;
	cout.flush();
#endif

	dVector3 res;	
	if (axis==1)
		dJointGetUniversalAxis1(jointID, res);
	else
		dJointGetUniversalAxis2(jointID, res);

	char axisName[100];
	snprintf(axisName, 100, "%i", axis);
	return this->Joint::getName() + axisName;
}

dReal DoubleJoint::getAngle(short axis)
{
#ifdef TRACING
	cout << "getAngle(" <<  axis << ")"  << endl;
	cout.flush();
#endif

	if (axis==1)
		return dJointGetUniversalAngle1(jointID);
	else
		return dJointGetUniversalAngle2(jointID);
}

void DoubleJoint::setVelocity(dReal vel, short axis)
{
#ifdef TRACING
	cout << "setVelocity(" <<  vel << ", " << axis << ")" << endl;
	cout.flush();
#endif
	
	if (axis==1)
	{
		dJointSetAMotorParam(motorID, dParamFMax, (float)parms.fMax); 
		dJointSetAMotorParam(motorID, dParamVel, vel);
	}
	else
	{
		dJointSetAMotorParam(motorID, dParamFMax2, (float)parms.fMax); 
		dJointSetAMotorParam(motorID, dParamVel2, vel);
	}
}

dReal DoubleJoint::getVelocity(short axis)
{
#ifdef TRACING
	cout << "getVelocity(" <<  axis << ")" << endl;
	cout.flush();
#endif

	if (axis==1)
	{
		return dJointGetAMotorParam(motorID, dParamVel);
	}
	else
	{
		return dJointGetAMotorParam(motorID, dParamVel2);
	}
}

void DoubleJoint::setTorque(dReal torque, short axis, dReal friction)
{
#ifdef TRACING
	cout << "setTorque(" << torque << ", " << axis << ", " << friction << ")" << endl;
	cout.flush();
#endif
	if (axis==1)
	{		
		dJointAddAMotorTorques(motorID, torque, 0, 0);
		dJointSetAMotorParam(motorID, dParamFMax, friction);
	}
	else
	{		
		dJointAddAMotorTorques(motorID, 0, torque, 0);
		dJointSetAMotorParam(motorID, dParamFMax2, friction);
	}
}

dReal DoubleJoint::getTorque(short axis)
{
#ifdef TRACING
	cout << "getTorque(" << axis << ")" << endl;
	cout.flush();
#endif
	
	dVector3 _axis;
	if (axis==1)
		dJointGetUniversalAxis1(jointID, _axis);
	else
		dJointGetUniversalAxis2(jointID, _axis);

	dJointFeedback *fb=dJointGetFeedback(motorID);
	return _axis[0]*fb->t1[0]+_axis[1]*fb->t1[1]+_axis[2]*fb->t1[2]-
		(_axis[0]*fb->t2[0]+_axis[1]*fb->t2[1]+_axis[2]*fb->t2[2]);
}

void DoubleJoint::setAngle(double angle, short axis) 
{
#ifdef TRACING
	cout << "setAngle(" << angle << ", " << axis << ")" << endl;
	cout.flush();
#endif
	motor[axis-1].controllerStep(angle, getAngle(axis));
}

void DoubleJoint::act(Method method)
{
	for (int i=1; i<3; i++)
	{

		double friction=parms.jointFriction[motor[i-1].getReductionType()];

#ifdef FRICTION_TYPE_STATIC_DYNAMIC
		if (getVelocity(i)>0.1)
			friction=parms.dynamicFriction;
#endif

		if (method==velocity)
			setVelocity(motor[i-1].getSpeed(), i);
		else
			setTorque(motor[i-1].getTargetTorque(getVelocity(i)), i, (float)friction);
	}
}

void DoubleJoint::setPIDParams(PIDController::PIDParams pidParams, short axis, Joint::Method method)
{
	motor[axis-1].setPIDParams(pidParams, method);
}
