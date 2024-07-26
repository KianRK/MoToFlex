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
#ifndef DRAWSTUFF_TEXTURE_PATH
#ifdef WIN32
#define DRAWSTUFF_TEXTURE_PATH "ode/drawstuff/textures"
#else
#define DRAWSTUFF_TEXTURE_PATH "ode/drawstuff/textures"
#endif
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

#include <genom.h>
#include "adapter/angleadapter.h"

extern Genom parms;
extern bool running;
extern bool quiet;
extern AngleAdapter *behaviour[NUM_OF_NAOS];

bool initSimulation(const char *, bool);
void quitSimulation();
void simLoop (int pause);
