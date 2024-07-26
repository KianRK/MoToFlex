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
 * @file doublejoint.h
 * Joint with two degrees of freedom.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */


#pragma once

#include "simulation.h"
#include "box.h"
#include "motor.h"
#include "joint.h"

/**
* @class DoubleJoint
* Joint with two degrees of freedom.
*/
class DoubleJoint : public Joint
{
public:
	/**
	 * Returns the measured angle.
	 */
	dReal getAngle(short axis);

	/**
	 * Returns the measured torque.
	 */
	dReal getTorque(short axis);

	/**
	 * Sets the desired velocity of an axis.
	 * \param vel The velocity.
	 * \param axis The axis.
	 */
	void setVelocity(dReal vel, short axis);

	/**
	 * Returns the velocity of an axis.
	 * \param axis Number of the axis (the first or the second one).
	 */
	dReal getVelocity(short axis);

	/**
	 * Set the torque which is applied to the connected bodies.
	 * \param torque The torque to be applied.
	 * \param axis Number of the axis (the first or the second one).
	 * \param friction The friction of the axis.
	 */
	void setTorque(dReal torque, short axis, dReal friction);

	/**
	 * Set the desired angle which is not necessarily the reached one.
	 * \param angle The target angle.
	 * \axis Number of the axis (the first or the second one).
	 */
	void setAngle(double angle, short axis);

	/**
	 * Execute the joint controller with the given method. 
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void act(Method method);

	/**
	 * Set p i and d value of the controller and set the method of joint control.
	 * \param pidParams Array of pid values, number depends on motorized joints.
	 * \param axis Number of the axis (the first or the second one).
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void setPIDParams(PIDController::PIDParams pidParams, short axis, Joint::Method method);

	/**
	 * Create the corresponding joint within the ODE environment.
	 * \param world The ODE-ID of the world.
	 * \param body1 Connect this body with body2
	 * \param body2 Connect this body with body1
	 * \param anchor Position of the axes.
	 * \param axis1 Direction of the first axis.
	 * \param axis2 Direction of the second axis.
	 * \param stiffness The nao specific stiffness factor.
	 * \param reductionType1 Reduction factor of the first gear type (two gear types possible).
	 * \param reductionType2 Reduction factor of the second gear type (two gear types possible).
	 */
	void createPhysics(dWorldID world,
		Box &body1,
		Box &body2,
		dReal anchor[],
		dReal axis1[],
		dReal axis2[],
		float stiffness=1.0,
		short reductionType1=-1,
		short reductionType2=-1);

	void destroyPhysics();

	/**
	 * Returns the name of the joint.
	 */
	string getName(short axis);

private:
	Motor motor[2];
};
