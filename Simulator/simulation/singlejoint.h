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
 * @file singlejoint.h
 * Joint with one degree of freedom.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "simulation.h"
#include "box.h"
#include "pidcontroller.h"
#include "joint.h"
#include "motor.h"

/**
* @class SingleJoint
* Joint with one degree of freedom.
*/
class SingleJoint : public Joint
{
public:
	/**
	 * Returns the measured angle.
	 */
	dReal getAngle();

	/**
	 * Returns the measured torque.
	 */
	dReal getTorque();

	/**
	 * Sets the desired velocity of an axis.
	 * \param vel The velocity.
	 */
	void setVelocity(dReal vel);

	/**
	 * Set the torque which is applied to the connected bodies.
	 * \param torque The torque to be applied.
	 */
	void setTorque(dReal torque);

	/**
	 * Returns the velocity of an axis.
	 * \param axis Number of the axis (the first or the second one).
	 */
	dReal getVelocity();

	/**
	 * Set the desired angle which is not necessarily the reached one.
	 * \param angle The target angle.
	 */
	void setAngle(double angle);

	/**
	 * Execute the joint controller with the given method. 
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void act(Method method);

	/**
	 * Set p i and d value of the controller and set the method of joint control.
	 * \param pidParams Array of pid values, number depends on motorized joints.
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void setPIDParams(PIDController::PIDParams pidParams, Joint::Method method);

	/**
	 * Create the corresponding joint within the ODE environment.
	 * \param world The ODE-ID of the world.
	 * \param body1 Connect this body with body2
	 * \param body2 Connect this body with body1
	 * \param anchor Position of the axes.
	 * \param axis Direction of the axis.
	 * \param stiffness The nao specific stiffness factor.
	 * \param reductionType Reduction factor.
	 */
	void createPhysics(dWorldID world,
		Box body1,
		Box body2,
		dReal anchor[],
		dReal axis[],
		short reductionType,
		float stiffness);

	void destroyPhysics();

	Motor motor;
};
