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
#include "flexiblebox.h"
#include <genom.h>

extern Genom parms;

FlexibleBox::FlexibleBox(void)
{
	boxes=NULL;
	joints=NULL;
}

FlexibleBox::~FlexibleBox(void)
{
	if (boxes!=NULL)
		delete[] boxes;

	if (joints!=NULL)
		delete[] joints;
}

void FlexibleBox::draw(double yOffset)
{
	for (int i=0; i<numOfBoxes; i++)
		boxes[i].draw(yOffset);
}
 
void FlexibleBox::createPhysics(dWorldID world,
						dSpaceID space,
						dReal pos[],
						dReal size[],
						dReal flexibleAxis[],
						dReal kg,
						int numOfElements)
{
	dReal xAxis[]={1, 0, 0};
	dReal yAxis[]={0, 1, 0};
	dReal sizeStep[3];
	dReal boxPos[3];
	dReal boxSize[3];

	numOfBoxes=numOfElements;
	weight=kg;

	for (int i=0; i<3; i++)
	{
		if (flexibleAxis[i]!=0)
			boxSize[i]=sizeStep[i]=size[i]/numOfElements;
		else
		{
			sizeStep[i]=0;
			boxSize[i]=size[i];
		}

		boxPos[i]=pos[i]-flexibleAxis[i]*size[i]/2+flexibleAxis[i]*boxSize[i]/2;
	}

	if (boxes == NULL)
		boxes=new Box[numOfElements];
	if (joints == NULL)
		joints=new DoubleJoint[numOfElements-1];
	kg/=numOfElements;

	for (int i=0; i<numOfElements; i++)
	{
		boxes[i].setColor(1.0f, 1.0f, 1.0f);
		boxes[i].createPhysics(world, space, boxPos, boxSize, kg);
		for (int j=0; j<3; j++)
			boxPos[j]+=sizeStep[j];
	}

	for (int i=0; i<3; i++)
		boxPos[i]=pos[i]-flexibleAxis[i]*size[i]/2+flexibleAxis[i]*boxSize[i];

	for (int i=0; i<numOfElements-1; i++)
	{
		joints[i].createPhysics(world,
			boxes[i],
			boxes[i+1], 
			boxPos,
			xAxis,
			yAxis);
		for (int j=0; j<3; j++)
			boxPos[j]+=sizeStep[j];
	}

}

Box *FlexibleBox::getBox(int index)
{
	return &boxes[index];
}

void FlexibleBox::act(dReal D)
{
	for (int i=0; i<numOfBoxes-1; i++)
	{
		for (int j=1; j<3; j++)
		{
			dReal angle=joints[i].getAngle(j);
			joints[i].setVelocity(0, j);
			joints[i].setTorque(-D*angle, j, (float)parms.flexibleFriction);
		}
	}
}

Vector3<double> FlexibleBox::getCoM()
{
	Vector3<double> CoM_WCS;

	for (int i=0; i<numOfBoxes; i++)
		CoM_WCS+=boxes[i].getPosition()/numOfBoxes;

	return CoM_WCS;
}