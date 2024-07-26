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
#include "singlejoint.h"
#include <genom.h>
#include "eaconfig.h"
#include <csvlogger.h>

#ifdef TRACING
#include <iostream>
using namespace std;
#endif

extern Genom parms;



void SingleJoint::createPhysics(dWorldID world,
								Box body1,
								Box body2,
								dReal anchor[],
								dReal axis[],
								short reductionType,
								float stiffness)
{
#ifdef TRACING
	cout << "createPhysics()" << endl;
	cout.flush();
#endif
	motor.init();
	jointID=dJointCreateHinge(world, 0);
	dJointAttach(jointID, body1.getID(), body2.getID());
	dJointSetHingeAnchor(jointID, anchor[0], anchor[1], anchor[2]);
	dJointSetHingeAxis(jointID, axis[0], axis[1], axis[2]);

	motorID=dJointCreateAMotor(world, 0);
	dJointAttach(motorID,  body1.getID(), body2.getID());
	dJointSetAMotorMode(motorID, dAMotorUser);
	dJointSetAMotorNumAxes(motorID, 1);
	dJointSetAMotorAxis(motorID, 0, 1, axis[0], axis[1], axis[2]);
	dJointSetFeedback(motorID, &fb);
	motor.setReductionType(reductionType);
	motor.stiffness=stiffness;
}

void SingleJoint::destroyPhysics()
{
	dJointDestroy(jointID);
	dJointDestroy(motorID);
}

dReal SingleJoint::getAngle()
{
	return dJointGetHingeAngle(jointID);
}

void SingleJoint::setVelocity(dReal vel)
{
#ifdef TRACING
	cout << "setVelocity(" << vel << ")" << endl;
	cout.flush();
#endif
	dJointSetAMotorParam(motorID, dParamFMax, (float)parms.fMax); 
	dJointSetAMotorParam(motorID, dParamVel, vel);
}

void SingleJoint::setTorque(dReal torque)
{
#ifdef TRACING
	cout << "setTorque(" << torque << ")" << endl;
	cout.flush();
#endif
#ifdef FRICTION_TYPE_STATIC_DYNAMIC
	if (getVelocity()<0.1)
		dJointSetAMotorParam(motorID, dParamFMax, parms.jointFriction[motor.reductionType]);
	else
		dJointSetAMotorParam(motorID, dParamFMax, parms.dynamicFriction);
#endif
#ifdef FRICTION_TYPE_PAPER
	dJointSetAMotorParam(motorID, dParamFMax, (float)parms.jointFriction[motor.getReductionType()]);
#endif
	dJointAddAMotorTorques(motorID, torque, torque, torque);
}

dReal SingleJoint::getTorque()
{
	dVector3 _axis;
	dJointGetHingeAxis(jointID, _axis);
	dJointFeedback *fb=dJointGetFeedback(motorID);
	return _axis[0]*fb->t1[0]+_axis[1]*fb->t1[1]+_axis[2]*fb->t1[2];
}


dReal SingleJoint::getVelocity()
{
	return dJointGetAMotorParam(motorID, dParamVel);
}

void SingleJoint::setAngle(double angle) 
{
#ifdef TRACING
	cout << "setAngle(" << angle << ")" << endl;
	cout.flush();
#endif
	motor.controllerStep(angle, getAngle());
}

void SingleJoint::act(Method method)
{
 	if (method==velocity)
		setVelocity(motor.getSpeed());
	else
		setTorque(motor.getTargetTorque(getVelocity()));

}

void SingleJoint::setPIDParams(PIDController::PIDParams pidParams, Joint::Method method)
{
	motor.setPIDParams(pidParams, method);
}


