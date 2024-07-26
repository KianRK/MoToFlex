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
 * @file robot.h
 * Base class for all robots.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "../simulation/joint.h"
#include <string>
#include "math/Matrix.h"

using namespace std;

/**
 * @class Robot
 * Base class for all robots.
 */
class Robot
{
public:

	/** 
	 * Returns the name of the robot.
	 * \return Name of the robot.
	 */
	string getName();

	/** 
	 * Orientation of a box of the robot.
	 * \param boxNum Number of the box.
	 * \param orientation Filled with 3 angles: Between vector (0, 1, 0)
	 * and plane with normal vector (0, 0, 1), (1, 0, 0) and (0, 0, 1),
	 * (1, 0, 0) and (0, 1, 0)
	 */
	virtual void getOrientation(int boxNum, float *orientation)=0;

	/** 
	 * Orientation of the body.
	 * \param orientation Filled with 3 angles. See getOrientation(int boxNum, float *orientation)
	 * for details.
	 */
	virtual void getOrientation(float *orientation)=0;

	/** 
	 * Position of a box of the robot.
	 * \param boxNum Number of the box.
	 * \param position Position coordinates (x, y, z).
	 */
	virtual void getPosition(int boxNum, float *position)=0;

	/** 
	 * Position of the body,
	 * \param position Position coordinates (x, y, z).
	 */
	virtual void getPosition(float *position)=0;

	/**
	 * Retrieves velocity of the body.
	 * \param position Array of 3 elements to be filled with the velocity.
	 */
	void getVelocity(dReal *velocity);

	/**
	 * Retrieves position of the body.
	 * \return Vector3 of the position.
	 */
	Vector3<double> getVelocity();

	/** 
	 * Returns the joint instance.
	 * \param jointID ID of the joint.
	 * \return The joint instance.
	 */
	virtual Joint *getJoint(int jointID)=0;

	/** 
	 * Retrieves the actual angles.
	 * \param angles Filled with getNumOfJoints() angles.
	 */
	virtual void getAngles(float angles[])=0;

	/** 
	 * Calculates the actual center of mass of the robot in the
	 * robot coordinate system.
	 * \return The center of mass position (x, y, z).
	 */
	virtual Vector3<double> getCoM()=0;		

	/** 
	 * Returns the values measured by the acceleration sensor
	 * \return Vector of the acceleration of along axes x, y and z.
	 */
	virtual Vector3<double> getAcc()=0;		// Values from inertia sensor

	/** 
	 * Returns the angle speed measured by the gyro.
	 * \return Angle speed around x, y and z.
	 */
	virtual Vector3<double> getGyr()=0;		

	/** 
	 * Returns the mass of the robot.
	 * \return Mass of robot.
	 */
	virtual float getWeight()=0;	

	/** 
	 * Returns the number of moveable joints of the robot. Joints with 2 axes
	 * counts 2.
	 * \return Number of joints.
	 */
	virtual int getNumOfJoints()=0;

	/** 
	 * Returns the full state of the robot.
	 * See: http://ode.org/wiki/index.php/HOWTO_save_and_restore
	 * \return Number of joints.
	 */
	virtual void getState()=0;

	/** 
	 * Sets the full state of the robot. Can be used to continue
	 * the simulation. See also getState()
	 * \return Number of joints.
	 */
	virtual void setState()=0;


	Vector3<float> ankleToGround; /**< Vector (x, y and z) from ankle joint to ground */
	Vector3<float> kneeToAnkle; /**< Vector (x, y and z) from knee joint to ankle */
	Vector3<float> hipToKnee; /**< Vector (x, y and z) from hip joint to knee */
	Vector3<float> bodyToRightHip; /**< Vector (x, y and z) from body joint to right hip */
};