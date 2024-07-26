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
 * @file nao.h
 * Implementation of a Nao robot.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */


#pragma once

#include "../simulation/doublejoint.h"
#include "../simulation/singlejoint.h"
#include "../simulation/box.h"
#include "../simulation/flexiblebox.h"
#include "eaconfig.h"
#include "../adapter/robot.h"
#include "math/Vector3.h"
#include "math/Matrix.h"

#ifndef MOVEABLE_ARMS
#define NUM_OF_JOINTS	10
#else
#define NUM_OF_JOINTS	14
#endif

/**
* @class Nao
* A Nao robot for the simulation.
*/
class Nao : public Robot
{
public:
	/** 
	 * Constructor.
	 */
	Nao();

	/** Possible action types. */
	enum ActionType {walk, specialAction};

	/** Enumeration of legs */
	enum {leftLeg, rightLeg, numOfLegs};

	/** Enumeration of arms */
	enum {leftArm, rightArm, numOfArms};

	enum {body,
		footLeft,
		footRight,
#ifdef ARMS
		armLeft,
		armRight,
		head,
#endif
		numOfBoxes};

	enum {upperLegLeft, upperLegRight, lowerLegLeft, lowerLegRight, numOfFlexBoxes};
	enum {lHipRoll, lHipPitch, lKneePitch, lAnklePitch, 
		lAnkleRoll, rHipRoll, rHipPitch, rKneePitch, 
		rAnklePitch, rAnkleRoll,
#ifdef MOVEABLE_ARMS
		leftArmRoll, leftArmPitch,
		rightArmRoll, rightArmPitch,
#endif
		numOfMotors};

	/**
	 * Create the Nao.
	 * \param world The ODE-ID of the world.
	 * \param space The ODE-ID of the space.
	 * \param actionType Type of the motion, walk or other motion.
	 * \param name Name of the Nao.
	 */
	void create(dWorldID world,
		dSpaceID space,
		ActionType actionType,
		string name);

	/**
	 * Stops the simulation of the Nao
	 */
	void stop();


	/**
	 * Draw the Nao with an offset to avoid drawing Naos of multiple space at the same place.
	 * \param yOffset Offset along y axis.
	 */
	void draw(double yOffset);

	/**
	 * Set target angles.
	 * \param angles The target angles. Number depends on used motorized joints.
	 */
	void setAngles(float angles[]);

	/**
	 * Get the measured angles.
	 * \param angles Array to fill. Number depends on used motorized joints.
	 */
	void getAngles(float angles[]);

	/**
	 * Set p i and d value of the controller and set the method of joint control.
	 * \param params Array of pid values, number depends on motorized joints.
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void setPIDParams(PIDController::PIDParams params[], Joint::Method method);

	/**
	 * Set p i and d value of the controller and set the method of joint control.
	 * \param p Same p value for every joint.
	 * \param i Same i value for every joint.
	 * \param d Same d value for every joint.
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void setAllPIDParams(float p, float i, float d, Joint::Method method);

	/**
	 * Execute the joint controller with the given method. 
	 * \param method Method used for joint control. Speed control leads to perfect angles, torque leads to more realizm.
	 */
	void act(Joint::Method method);

	/**
	 * Retrieves the orientation of a box.
	 * \param boxNum ID of the box.
	 * \param orientation The orientation of the box. For details see description of Box::getOrientation().
	 */
	void getOrientation(int boxNum, dReal *orientation);

	/**
	 * Retrieves the orientation of the body.
	 * \param orientation The orientation of the box. For details see description of Box::getOrientation().
	 */
	void getOrientation(dReal *orientation);

	/**
	 * Returns the orientation matrix of the body.
	 */
	RotationMatrix getOrientation();

	/**
	 * Retrieves position of a box.
	 * \param boxNum ID of the box.
	 * \param position Array of 3 elements to be filled with the position coordinates.
	 */
	void getPosition(int boxNum, dReal *position);


	/**
	 * Retrieves position of the body.
	 * \param position Array of 3 elements to be filled with the position coordinates.
	 */
	void getPosition(dReal *position);

	/**
	 * Retrieves velocity of the body.
	 * \param position Array of 3 elements to be filled with the velocity.
	 */
	void getVelocity(dReal *velocity);

	/**
	 * Retrieves position of the body.
	 * \return Vector3 of the position.
	 */
	Vector3<double> getPosition();

	/**
	 * Retrieves the size of the body.
	 */
	void getSize(int boxNum, dReal *size);

	/**
	 * Adds the given force to the given box.
	 */
	void addForce(int boxNum, dReal fx, dReal fy, dReal fz);

	/**
	 * Retrieves position of the body.
	 * \return Vector3 of the position.
	 */
	Vector3<double> getVelocity();

	/**
	 * Retrieves the flex box.
	 * \return Vector3 of the position.
	 */
	FlexibleBox* getFlexbox(int flexboxID) { return &flexboxes[flexboxID]; }

	/**
	 * Returns the Joint object.
	 * \param jointID ID of the joint.
	 */
	Joint *getJoint(int jointID);

	/**
	 * Return the overall center of mass of the robot.
	 */
	Vector3<double> getCoM();

	/**
	 * Returns the acceleration of the body.
	 */
	Vector3<double> getAcc();	

	/**
	 * Returns the gyroscope values of the body.
	 */
	Vector3<double> getGyr();

	/**
	 * Returns the mass of the robot.
	 */
	float getWeight();

	/**
	 * Returns the ODE-ID of the given box number.
	 * \param boxNum The number of the box.
	 */
	dBodyID getBodyID(int boxNum);

	/**
	 * Returns the name of the robot.
	 */
	string getName() { return name; }

	/**
	 * Returns the number of joints.
	 */
	int getNumOfJoints();

	/**
	 * Logs some data of the robot.
	 */
	void log();

	/** 
	 * Returns the full state of the robot.
	 * See: http://ode.org/wiki/index.php/HOWTO_save_and_restore
	 * \return Number of joints.
	 */
	void getState() {};

	/** 
	 * Sets the full state of the robot. Can be used to continue
	 * the simulation. See also getState()
	 * \return Number of joints.
	 */
	void setState() {};
private:
	bool created;
	ActionType actionType;
	string name;
	DoubleJoint hip[numOfLegs], ankle[numOfLegs];
#ifdef ARMS
	DoubleJoint shoulder[numOfArms];
#endif
	SingleJoint knee[numOfLegs];
	Box boxes[numOfBoxes];
	FlexibleBox flexboxes[numOfFlexBoxes];
};														   

