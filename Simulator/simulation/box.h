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
#include <drawstuff/drawstuff.h>
#include "simulation.h"
#include "math/Matrix.h"

class Box
{
public:
	Box();
	void setColor(float r, float g, float b);
	void draw(double yOffset);
	void createPhysics(dWorldID world,
		dSpaceID space,
		dReal pos[],
		dReal size[],
		dReal kg);
	void destroyPhysics();

	dBodyID getID();
	RotationMatrix getRotationMatrix();
	void getOrientation(dReal *rotation);
	void getPosition(dReal *pos);
	void getVelocity(dReal *vel);
	void getForce(dReal *force);
	void addForce(dReal fx, dReal fy, dReal fz);
	float getWeight() { return (float)m.mass; }
	dReal* getSize() { return size; }
	Vector3<double> getPosition();
	Vector3<double> getVelocity();
private:
	dBodyID boxID;
	dGeomID geomID;
	const dReal *pos, *rot;
	dReal size[3];
	float r, g, b;
	dMass m;

};
