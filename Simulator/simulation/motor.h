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
/**
 * @file motor.h
 * Simulation of a motor with gears.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */


#pragma once
#include <ode/ode.h>
#include "pidcontroller.h"
#include "../filter/FastFilter.h"
#include "eaconfig.h"
#include "joint.h"

/**
* @class Motor
* Simulation of a motor with gears.
*/
class Motor
{
public:
	/** 
	 * Constructor.
	 */
	Motor();

	/** 
	 * Destructor.
	 */
	~Motor(void);

	/**
	 * Execute one step of the joint controlling pid controller.
	 * \param target Target angle.
	 * \param actual Measured angle.
	 */
	void controllerStep(double target, double actual);

	/**
	 * Init.
	 */
	void init();

	/**
	 * Set p i and d value of the controller and set the method of joint control.
	 * \param pidParams The p i d parameters.
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void setPIDParams(PIDController::PIDParams pidParams, Joint::Method method);

	/**
	 * Set the type of the gears. Two types possible.
	 */
	void setReductionType(int type);

	/**
	 * Returns the type of the gears.
	 */
	int getReductionType() { return reductionType; }

	/**
	 * Do one simulation step of the motor and the gears
	 * \param angleSpeed Current speed of the axis.
	 */
	dReal getTargetTorque(double angleSpeed);

	/**
	 * Returns the speed of the axis.
	 */
	dReal getSpeed();

	char sign(double x) { return ((x < 0.0f) ? -1 : +1);}

	/** Stiffness of the joint (Nao specific parameter). */
	double stiffness;

#ifdef FLEXIBLE_TEST
	float in, out;
#endif
private:
	dReal realTorque;
	short reductionType;
	FastFilter torqueFilter;
	PIDController pidc, velc;
	dReal controllerVolt, controllerSpeed, curVolt, lastVolt, startVolt, dVolt;
	int t;
	char lastTorqueSign;
	double toWait;
	double massPos, massSpeed, hullPos;
};
