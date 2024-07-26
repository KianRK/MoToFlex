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
#include "motor.h"
#include <genom.h>
#include <math.h>
#include "../eaconfig.h"

#ifdef FLEXIBLE_TEST
float r, g, b;
#endif

#define TORQUE_FILTER_LEN 5

extern Genom parms;

Motor::Motor(void)
{
}

void Motor::init()
{
	torqueFilter.createBuffer(TORQUE_FILTER_LEN);
	massPos=0;
	massSpeed=0;
	hullPos=0;
	realTorque=0;
	controllerSpeed=0;
	curVolt=0;
	lastVolt=0;
	startVolt=0;
	dVolt=0;
	lastTorqueSign=0;
}

Motor::~Motor(void)
{
}

void Motor::setReductionType(int type)
{
	reductionType=type;
	toWait=parms.tolerance[reductionType];
}

void Motor::setPIDParams(PIDController::PIDParams pidParams, Joint::Method method)
{
	if (method==Joint::torque)
		pidc.setParams(pidParams);
	else
		velc.setParams(pidParams);
};


dReal Motor::getSpeed()
{
	return controllerSpeed;
}

dReal Motor::getTargetTorque(double angleSpeed)
{
	dReal targetTorque;

#ifdef IMPEDANCE
	curVolt+=dVolt;
	dVolt=(float)((controllerVolt-curVolt)*(1-exp(-(parms.frameLen)/(parms.L/parms.R))));
#else
	curVolt+=dVolt;
	dVolt=(controllerVolt-curVolt)*1;
#endif	

	dReal current=(float)(curVolt/parms.R);
	current*=stiffness;

	targetTorque=(float)(parms.K*current);



	// This is an other way than in "Humanoid realistic simulator". Here
	// the speed independent part of the friction is handeled by the ODE itself,
	// since this friction also appears when we do not excert any torque.
	// The friction appears only, when the gear is moving.
	// B also includes the back emf voltage constant
#ifdef FRICTION_TYPE_PAPER
	targetTorque-=(angleSpeed*(parms.B[reductionType])); // do not convert this to float
#endif

	if (reductionType!=-1)
		targetTorque*=parms.reduction[reductionType];

#ifdef SIMPLE_TOLERANCE_GEARS
	char torqueSign=sign(targetTorque);
	if (torqueSign!=lastTorqueSign)
	{
		toWait-=parms.frameLen;
		if (toWait<=0)
		{
			lastTorqueSign=torqueSign;
			toWait=parms.tolerance[reductionType];
		}
		return 0;
	}

	return targetTorque;
#endif


#ifdef FLEXIBLE_TOLERANCE_GEARS
	double springTorque, resultTorque=0;
	for (int i=0; i<100; i++)
	{
		massPos+=massSpeed*(parms.frameLen/100);
		hullPos=0;
		if (massPos-parms.length[reductionType]>hullPos)
			hullPos=massPos-parms.length[reductionType];
		if (massPos<hullPos)
			hullPos=massPos;
		springTorque=parms.gearSc[reductionType]*hullPos;
		massSpeed+=(targetTorque-springTorque-parms.innerDamping[reductionType]*massSpeed)/parms.mass[reductionType]*(parms.frameLen/100);
		resultTorque+=springTorque;
	}

#ifdef FLEXIBLE_TEST
	in=targetTorque;
	out=(float)(resultTorque/100);
	if (hullPos!=massPos-parms.length[reductionType] && hullPos!=massPos)
	{ r=0; g=0; b=1; }
	else
	{
		b=0;
		r=(float)(fabs(hullPos)*200000);
		g=1-r;
	}
#endif
	return (float)(resultTorque/100);
#else
	return targetTorque;
#endif
	
}

void Motor::controllerStep(double target, double actual)
{

	controllerVolt=(float)pidc.doIt(target, actual);
	controllerSpeed=(float)velc.doIt(target, actual);

#ifdef MAX_VOLT
	if (controllerVolt>parms.maxVolt)
		controllerVolt=(float)parms.maxVolt;
#endif
}

