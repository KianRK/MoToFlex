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
#include "world.h"
#include "eaconfig.h"
#include "pidcontroller.h"

World worlds[NUM_OF_NAOS];

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

bool World::step()
{
	for (int k=0; k<ALLOWED_BODIES+1; k++)
		this->contact[k]=false;
	
	ok=true;
	dSpaceCollide (space,this, collisionCallback);

	// if a crash happens here it does not mean, that there is a bug. The most
	// likely reason are crap parameters.
#ifdef LINUX64
	// For whatever reason, on a 64 Linux (and only there), the solver of 
	// dWorldStep fails sometimes, this one works, hopefully giving
	// similar results. See https://groups.google.com/g/ode-users/c/cOYl7WF7oXo
	dWorldQuickStep (world, parms.frameLen);
#else
	dWorldStep (world, parms.frameLen);
#endif
	dJointGroupEmpty (contactgroup);
	return ok;
}

double offsetL=0, offsetR=0;
void World::collisionCallback (void *data, dGeomID o1, dGeomID o2)
{
    // pData is the "this" that was passed into dSpaceCollide
    World* pThis = (World*) data;
    if (pThis)
		pThis->collisionCallback (o1, o2);
}

void World::init(Genom &parms,
				 Nao::ActionType actionType,
				 string name)
{
	this->parms=parms;

	world = dWorldCreate();			
	space = dHashSpaceCreate (0);			//CollisionCheck-Welt erzeugen; 0 - SpaceID
	contactgroup = dJointGroupCreate (0);		//ContactJointGroup erzeugen; 0 - SpaceID
	dWorldSetGravity (world,0,0,-9.81f);		//Gravitation setzen
	ground = dCreatePlane (space,0,0,1,0);	//Grundebene erzeugen; space - Raum, Gleichung fuer die Ebene: 0x+0y+1z=0
	dWorldSetERP (world, (float)parms.erp);
	dWorldSetCFM (world, (float)parms.cfm);

	nao.create(world, space, actionType, name);
	nao.setAllPIDParams((float)parms.p, (float)parms.i, (float)parms.d, Joint::torque);
	nao.setAllPIDParams(10, 0, 0, Joint::velocity);

	// Es duerfen nur der Boden und die Fuesse an Kontakten beteiligt sein
	allowedBodies[0]=0;
	allowedBodies[1]=nao.getBodyID(Nao::footLeft);
	allowedBodies[2]=nao.getBodyID(Nao::footRight);

}

void World::collisionCallback (dGeomID o1, dGeomID o2)
{
	int i,n;

	offsetR=0;
	offsetL=0;
	static dGeomID ignore=(dGeomID)-1;

	if (o1==ignore || o2==ignore)
		return;

	// only collide things with the ground
	int g1 = (o1 == ground || o1 == ground_box);
	int g2 = (o2 == ground || o2 == ground_box);
	if (!(g1 ^ g2)) return;

	const int N = 10;
	dContact contact[N];
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n > 0) {
		for (i=0; i<n; i++) {
			dBodyID id1=dGeomGetBody(contact[i].geom.g1);
			dBodyID id2=dGeomGetBody(contact[i].geom.g2);
			bool ok1=false, ok2=false;
			for (int k=0; k<ALLOWED_BODIES+1; k++)
			{
				if (id1==allowedBodies[k])
				{
					ok1=true;
					this->contact[k]=true;
					offsetL=0.03;
				}
				if (id2==allowedBodies[k])
				{
					ok2=true;
					this->contact[k]=true;
					offsetR=0.03;
				}	
			}
			if (!ok1 || !ok2)
			{
				// falling is not allowed, but the ea needs
				// to see differences in the angleFitness even 
				// when the robot falls down
				ok=false;
			}


			contact[i].surface.mode = 
#ifdef SLIPPING
				dContactSlip1 | dContactSlip2 |	
#endif
				dContactSoftERP | dContactSoftCFM | dContactApprox1;
			contact[i].surface.mu = (float)parms.mu;
#ifdef SLIPPING
			contact[i].surface.slip1 = parms.slip1;
			contact[i].surface.slip2 = parms.slip2;
#endif
			contact[i].surface.soft_erp = (float)parms.soft_erp;
			contact[i].surface.soft_cfm = (float)parms.soft_cfm;

			dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
			dJointAttach (c, id1, id2);
		}
	}
}
