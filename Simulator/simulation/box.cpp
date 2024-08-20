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
#include "box.h"


Box::Box()
{
	setColor(0.5f, 0.7f, 0.7f);
}

void Box::setColor(float r, float g, float b)
{
	this->r=r;
	this->g=g;
	this->b=b;
}

void Box::getBodyOrientationQuaternion(dReal *quat)
{
	const dReal *ori=dBodyGetQuaternion(boxID);
	quat[0]=ori[0];
	quat[1]=ori[1];
	quat[2]=ori[2];
	quat[3]=ori[3];
}

void Box::getPosition(dReal *pos)
{
	const dReal *tPos=dBodyGetPosition(boxID);
	pos[0]=tPos[0];
	pos[1]=tPos[1];
	pos[2]=tPos[2];
}

void Box::getVelocity(dReal *vel)
{
	const dReal *tVel=dBodyGetLinearVel(boxID);
	vel[0]=tVel[0];
	vel[1]=tVel[1];
	vel[2]=tVel[2];
}

void Box::getAngularVelocity(dReal *vel)
{
	const dReal *tVel=dBodyGetAngularVel(boxID);
	vel[0]=tVel[0];
	vel[1]=tVel[1];
	vel[2]=tVel[2];
}

void Box::getForce(dReal *force)
{
	const dReal *tForce=dBodyGetForce(boxID);
	force[0]=tForce[0];
	force[1]=tForce[1];
	force[2]=tForce[2];
}

void Box::addForce(dReal fx, dReal fy, dReal fz)
{
	dBodyAddForce(boxID, fx, fy, fz);
}

void Box::draw(double yOffset)
{
	pos=dBodyGetPosition(boxID);
	rot=dBodyGetRotation(boxID);
	dReal pos2[3];
	pos2[0]=pos[0]; pos2[1]=pos[1]; pos2[2]=pos[2];
	pos2[1]+=(float)yOffset;
	dsSetColor(r, g, b);
	dsDrawBox(pos2, rot, size);
}

void Box::createPhysics(dWorldID world,
						dSpaceID space,
						dReal pos[],
						dReal size[],
						dReal kg)
{
	this->size[0]=size[0];
	this->size[1]=size[1];
	this->size[2]=size[2];
	boxID = dBodyCreate (world);
	dBodySetPosition (boxID, pos[0], pos[1], pos[2]);

	dMassSetBoxTotal(&m, kg, size[0], size[1], size[2]);
	// Trying to move the center of gravity out of the center
	// results in strange behaviour. Even setting it to x=1000
	// results in the same behaviour as 0.1
	// Setting an inertia tensor seems to work, but we havent
	// something different to this

	dBodySetMass (boxID,&m);	
	geomID = dCreateBox (0, size[0], size[1], size[2]);
	dGeomSetBody (geomID,boxID);	
	dSpaceAdd (space, geomID);
}

void Box::destroyPhysics()
{
	dBodyDestroy(boxID);
}

dBodyID Box::getID()
{
	return boxID;
}

//testJoint.getAngle()
//0.45943737

//-		orimat	0x0591f664	float [12]
//		[0]	1.0000000	float
//		[1]	-0.00000000	float
//		[2]	0.00000000	float
//		[3]	0.00000000	float
//		[4]	0.00000000	float
//		[5]	0.89630264	float
//		[6]	0.44344288	float
//		[7]	0.00000000	float
//		[8]	-0.00000000	float
//		[9]	-0.44344288	float
//		[10]	0.89630264	float
//		[11]	0.00000000	float

//>> rotMatrixAroundX(0.45943737)
//
//ans =
//
//    1.0000         0         0
//         0    0.8963   -0.4434
//         0    0.4434    0.8963

RotationMatrix Box::getRotationMatrix()
{
	Vector3<double> c[3];
	const dReal *ori=dBodyGetRotation (boxID);

	c[0].x=ori[0]; c[0].y=ori[1]; c[0].z=ori[2];
	c[1].x=ori[4]; c[1].y=ori[5]; c[1].z=ori[6];
	c[2].x=ori[8]; c[2].y=ori[9]; c[2].z=ori[10];

	RotationMatrix rotmat(c[0], c[1], c[2]);
	return rotmat;
	
}

Vector3<double> Box::getPosition()
{
	dReal p[4];
	getPosition(p);
	return Vector3<double>(p[0], p[1], p[2]);
}

Vector3<double> Box::getVelocity()
{
	dReal v[4];
	getVelocity(v);
	return Vector3<double>(v[0], v[1], v[2]);
}

void Box::getOrientation(dReal *rotation)
{
	// see notes for details...

	const dReal *ori=dBodyGetRotation (boxID);	
	dReal a32=ori[9], a21=ori[4], a13=ori[2];
	// Roll
	rotation[0]=asin(a32);

	// Pitch
	rotation[1]=asin(a13);

	// Direction
	rotation[2]=asin(a21);
}
