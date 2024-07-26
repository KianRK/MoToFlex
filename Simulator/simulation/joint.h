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
 * @file joint.h
 * Base class for joints.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
#include "pidcontroller.h"
#include <ode/ode.h>
#include "../filter/FastFilter.h"
#include <string>

using namespace std;

/**
* @class DoubleJoint
* Base class for joints.
*/
class Joint
{
public:
	enum Method {velocity, torque};

	/**
	 * Set the name for the joint.
	 */
	void setName(string name) { this->name=name; }

	/**
	 * Returns the name of the joint.
	 */
	string getName() { return name; }

	/**
	 * Returns the position of the axis.
	 */
	void getAnchor(float *anchor)
	{
		memcpy(anchor, this->anchor, sizeof(float)*3);
	}

protected:
	dJointID jointID, motorID;
	dJointFeedback fb;
	string name;
	
private:
	float anchor[3];
};
