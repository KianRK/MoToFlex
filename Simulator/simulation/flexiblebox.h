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
 * @file flexiblebox.h
 * Simulation of a flexible box.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
#include "box.h"
#include "doublejoint.h"
#include "math/Vector3.h"

/**
* @class DoubleJoint
* Simulation of a flexible box.
*/
class FlexibleBox
{
public:	
	/** 
	 * Constructor.
	 */
	FlexibleBox(void);

	/** 
	 * Destructor.
	 */
	~FlexibleBox(void);

	/**
	 * Draw the box with an offset to avoid drawing Naos of multiple space at the same place.
	 * \param yOffset Offset along y axis.
	 */
	void draw(double yOffset);
	
	/**
	 * Create the box by inserting multiple boxes connected by damped springs.
	 * \param world The ODE-ID of the world.
	 * \param space The ODE-ID of the space.
	 * \param pos Position of the box.
	 * \param size Size of the box.
	 * \param flexibleAxis The ODE boxes will die arranged and connect along this axis.
	 * \param kg Mass of the box in kg.
	 * \param numOfElements Number of the ODE boxes which build this flexible box.
	 */
	void createPhysics(dWorldID world,
						dSpaceID space,
						dReal pos[],
						dReal size[],
						dReal flexibleAxis[],
						dReal kg,
						int numOfElements);

	/**
	 * Returns a specific Box.
	 */
	Box *getBox(int index);

	/**
	 * Execute the damped spring simulation.
	 */
	void act(dReal D);

	/**
	 * Returns the mass of the box.
	 */
	float getWeight() { return weight; }

	/**
	 * Returns the number of boxes in this flexbox.
	 */
	float getNumOfBoxes() { return numOfBoxes; }

	/**
	 * Returns the center of mass of the box.
	 */
	Vector3<double> getCoM();
private:
	Box *boxes;
	DoubleJoint *joints;
	int numOfBoxes;
	float weight;
};
