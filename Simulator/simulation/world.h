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
#pragma once
#include <ode/ode.h>
#include "../nao/nao.h"
#include <genom.h>

#define ALLOWED_BODIES	2

class World
{
public:

	/**
	 * Execute the physics simulation
	 * \return False, if the robot has fallen down.
	 */
	bool step();

	/**
	 * Initialize the physics simulation
	 * \param parms Parameters for simulation
	 * \param actionType How to control the joints.
	 * \param name Name of the Nao.
	 */
	void init(Genom &parms,
		Nao::ActionType actionType, 
		string name);

	/**
	 * Quit the simulation.
	 */
	void quit()
	{
		nao.stop();
		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
	}

	Nao nao;
    
	/**
	 * Is the foot in contact with the ground?
	 * \param i Number of foot.
	 */
	bool hasContact(int i) { return contact[i]; }

	bool ok;
private:
	static void collisionCallback (void *data, dGeomID o1, dGeomID o2);
	void collisionCallback (dGeomID o1, dGeomID o2);

	dWorldID world;
	dSpaceID space;
	dJointGroupID contactgroup;

	Genom parms;

	/** Only this bodies are allow to be in contact with the ground. Mostly feet are allowed, otherwise the robot has fallen down.*/
	dBodyID allowedBodies[ALLOWED_BODIES+1];
	bool contact[ALLOWED_BODIES+1];
	dGeomID ground;
	dGeomID ground_box;
};

extern World worlds[NUM_OF_NAOS];
