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
#include "simulation.h"
#include "joint.h"
#include <tools/anglereader.h>
#include <math.h>
#include "math/Matrix.h"
#include <iostream>
#include "../nao/nao.h"
#include "flexiblebox.h"
#include "../params/paths.h"
#include "../ea/anglefitness.h"
#include "../ea/speedfitness.h"
#include <csvlogger.h>
#include "world.h"
#include "eaconfig.h"
#include "adapter/anglefromcsv.h"
#include "adapter/anglefrompython.h"
#include "adapter/dortmundwalkingengine.h"

#define MAX_PATH_LENGTH 1024

#ifdef MATLAB_MEX_FILE
#include "mexFunction.h"
#pragma comment(lib, "libmx.lib")
#pragma comment(lib, "libmat.lib")
#pragma comment(lib, "libmex.lib")
#endif

typedef AngleFitness Fitness;

using namespace std;

Fitness angleFitness;
AngleReader *targetAngles[NUM_OF_NAOS];
AngleAdapter *behaviour[NUM_OF_NAOS];

Genom parms;
Paths paths;

extern double offsetL, offsetR;

#ifdef FLEXIBLE_TEST
extern float r, g, b;
double angle=0;
SingleJoint testJoint;
static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;
static dGeomID ground;
static dGeomID ground_box;
Box secondBox;

#ifdef FLEXBOX
FlexibleBox firstBox;
#else
Box firstBox;
#endif
#endif

#ifdef SOFT_HEADLESS
#define HEADLESS
#endif

 #ifndef HEADLESS
// start simulation - set viewpoint

static void start()
{
	static float xyz[3] = {0.5778f, -1.0322f, 0.5000f};
	static float hpr[3] = {118.5000f, -19.5000f, 0.0000f};
	dsSetViewpoint (xyz,hpr);
}


// called when a key pressed
static void command (int cmd)
{
	float xyz[3];
	float hpr[3];
	switch (cmd) {

  case ',':
	  break;

  case '.':
	  break;

  case ' ':
	  break;

  case 'v':
	  dsGetViewpoint (xyz,hpr);
	  printf("xyz={%.4f, %.4f, %.4f}, hpr={%.4f, %.4f, %.4f}\r\n", xyz[0], xyz[1], xyz[2], hpr[0], hpr[1], hpr[2]);
	  break;

	// Pressing a number in the simulation value leads to different viewing angles.
  case '1':
	  //xyz={0.7603, 0.0063, 0.0400}, hpr={180.0000, 12.5000, 0.0000}
	  xyz[0]=0.7603f; xyz[1]=0.0063f;	xyz[2]=0.0400f;
	  hpr[0]=180.0000f; hpr[1]=12.5000f; hpr[2]=0;
	  dsSetViewpoint (xyz,hpr);
	  break;
  case '2':
	  //xyz={0.6404, -0.2958, 0.1500}, hpr={133.0000, -12.5000, 0.0000}
	  xyz[0]=0.6404f; xyz[1]=-0.2958f;	xyz[2]=0.1500f;
	  hpr[0]=133.0000f; hpr[1]=-12.5000f; hpr[2]=0;
	  dsSetViewpoint (xyz,hpr);
	  break;

  case '3':
	  //xyz[3] = {0.5778, -1.0322, 0.5000}; hpr[3] = {118.5000, -19.5000, 0.0000};
	  xyz[0]=0.5778f; xyz[1]=-1.0322f;	xyz[2]=0.5000f;
	  hpr[0]=118.5000f; hpr[1]=-19.5000f; hpr[2]=0;
	  dsSetViewpoint (xyz,hpr);
	  break;

 case '4':
	  //xyz={0.9019, -1.1414, 0.5000}, hpr={132.5000, -10.5000, 0.0000}
	  xyz[0]=0.9019f; xyz[1]=-1.1414f;	xyz[2]=0.5000f;
	  hpr[0]=132.5000f; hpr[1]=-10.5000f; hpr[2]=0;
	  dsSetViewpoint (xyz,hpr);
	  break;
#ifdef FLEXIBLE_TEST
 case 'a':
	 angle=angle+0.05;
	 break;
 case 's':
	 angle=angle-0.05;
	 break;
#ifdef FLEXBOX
 case 'x':
	 dBodyAddForce(firstBox.getBox(19)->getID(), 0, -10, 0);
	 firstBox.getBox(19)->setColor(0, 0, 1);
	  break;
 case 'y':
	  dBodyAddForce(firstBox.getBox(19)->getID(), 0, 10, 0);
	  firstBox.getBox(19)->setColor(0, 1, 0);
	  break;
#endif

#endif
	}
}
#endif

int frame=0;
float angles[NUM_OF_NAOS+1][NUM_OF_JOINTS], lastAngles[NUM_OF_NAOS+1][NUM_OF_JOINTS], measuredAngles[NUM_OF_NAOS+1][NUM_OF_JOINTS];
float target[NUM_OF_NAOS+1][NUM_OF_JOINTS], lastTarget[NUM_OF_NAOS+1][NUM_OF_JOINTS];
bool initialized = false;
bool running = false;

#ifdef SIM_QUIET
bool quiet = true;
#else
bool quiet = false;
#endif

bool initSimulation(const char *configPath, bool from_python)
{
	if (initialized)
		return false;

	if (!paths.load(configPath))
		return false;

	memset(angles, 0, (NUM_OF_NAOS+1) * NUM_OF_JOINTS * sizeof(float));
	memset(target, 0, (NUM_OF_NAOS+1) * NUM_OF_JOINTS * sizeof(float));
	angleFitness.setPath(paths.fitnessPath.c_str());
	parms.load(paths.paramsPath.c_str());
	CSVLogger::basepath=paths.outputPath+"-";

	dInitODE();
	for (int i=0; i<NUM_OF_NAOS; i++)
	{
		if (from_python)
		{
			if (!quiet)
				printf("Using angles from python module.");
		}
		else
		{
			// Set this e.g. to DortmundWalkingEngine if another source for angles is desired
			behaviour[i] = new AngleFromCSV();
			behaviour[i]->setSettingsPath(paths.behPath[i].c_str());
		}

		targetAngles[i]=NULL;
#ifdef EA_MODE
		targetAngles[i]=new AngleReader(paths.targetPath[i].c_str());
#endif

#ifdef ANGLE_INTERPOLATION
			if (targetAngles[i]!=NULL)
				targetAngles[i]->getAngles(lastTarget[i], NUM_OF_JOINTS);
			behaviour[i]->getAngles(lastAngles[i], NUM_OF_JOINTS, worlds[i].nao);
#endif

		int start=paths.behPath[i].find_last_of("/\\");
		int end=paths.behPath[i].find_last_of(".");
		string name=paths.behPath[i].substr(start+1, end-start-1)+"-Output";
		if (i==4)
			worlds[i].init(parms, Nao::specialAction, name);
		else
			worlds[i].init(parms, Nao::walk, name);
	}
	frame=0;
	initialized = true;
	running = true;
	return true;
}

void quitSimulation()
{
	if (!initialized)
		return;
	for (int i=0; i<NUM_OF_NAOS; i++)
	{
		worlds[i].quit();
	}
	initialized = false;
	running = false;
	dCloseODE();
}

// The main simulation loop.
void simLoop (int pause)
{
	float interpolatedBehaviour[NUM_OF_NAOS+1][NUM_OF_JOINTS];
	float interpolatedTarget[NUM_OF_NAOS+1][NUM_OF_JOINTS];

	// The physics simulation can work with a shorter time step length than the PID controller
	// of the Nao. This is the number of physic frames between.
	int framesBetweenAngles=((int)(SOURCE_DT/parms.frameLen)+1);
	int behaviourFrame=frame/framesBetweenAngles;

	// Do the following for every Nao (multiple Naos per simulation possible, but only
	// in different physical worlds).
	for (int nao_num=0; nao_num<NUM_OF_NAOS; nao_num++)
	{

		// Read next target angles.
		if (frame%framesBetweenAngles==0)
		{
#ifdef ANGLE_INTERPOLATION
			for (int i=0; i<NUM_OF_JOINTS; i++)
			{
				lastAngles[nao_num][i]=angles[nao_num][i];
				lastTarget[nao_num][i]=target[nao_num][i];
			}
#endif		
			behaviour[nao_num]->getAngles(angles[nao_num], NUM_OF_JOINTS, worlds[nao_num].nao);
			if (targetAngles[nao_num]!=NULL)
				targetAngles[nao_num]->getAngles(target[nao_num], NUM_OF_JOINTS); 

		}

		bool controllerFrame=frame%(int)floor(((1.0f/parms.frameLen)/parms.controllerFreq)+0.5f)==0;

		// If it is time to execute the pid controller, do it by setting the desired angles.
		if (controllerFrame)
		{
#ifdef ANGLE_INTERPOLATION
			for (int i=0; i<NUM_OF_JOINTS; i++)
				interpolatedBehaviour[nao_num][i]=(float)(lastAngles[nao_num][i]+(angles[nao_num][i]-lastAngles[nao_num][i])*(frame%framesBetweenAngles)/(double)framesBetweenAngles);
			worlds[nao_num].nao.setAngles(interpolatedBehaviour[nao_num]);
#else
			worlds[nao_num].nao.setAngles(angles[nao_num]);
#endif
				
		}

		if (behaviourFrame==parms.measureStart*(0.02/SOURCE_DT))
#ifdef ANGLE_INTERPOLATION
			worlds[nao_num].nao.setAllPIDParams(1000, 0, 0, Joint::velocity);
#else
			worlds[nao_num].nao.setAllPIDParams(50, 0, 0, Joint::velocity);
#endif

#ifdef TORQUE_CONTROL

		// For stability of the simulation use the safe velocity method in the first
		// frames of the simulation to avoid big jumps in the desired angles.
		if (behaviourFrame<parms.measureStart)
			worlds[nao_num].nao.act(Joint::velocity);	
		else
			worlds[nao_num].nao.act(Joint::torque);	
#else
		worlds[nao_num].nao.act(Joint::velocity);
#endif
		
		if (behaviourFrame>parms.measureStart*(0.02/SOURCE_DT) && controllerFrame && targetAngles[nao_num]!=NULL)
		{
			worlds[nao_num].nao.getAngles(measuredAngles[nao_num]);

			dReal speed=0;
			/* Values of measuredAngles should be interpolated since they are measured at
			   50 Hz so they are expected to produce spasmodic behaviour of the robot */
#ifdef ANGLE_INTERPOLATION
			for (int i=0; i<NUM_OF_JOINTS; i++)
				interpolatedTarget[nao_num][i]=(float)(lastTarget[nao_num][i]+(target[nao_num][i]-lastTarget[nao_num][i])*(frame%framesBetweenAngles)/(double)framesBetweenAngles);
			angleFitness.addValues(interpolatedTarget[nao_num], measuredAngles[nao_num], NUM_OF_JOINTS, speed);
#else
			angleFitness.addValues(target[nao_num], measuredAngles[nao_num], NUM_OF_JOINTS, speed);
#endif
		}

	}

#ifdef EA_MODE
	// Fitness calculation for the evolutionary algorithm
	if (behaviourFrame==parms.measureStop*(0.02/SOURCE_DT))
	{
		angleFitness.done(frame);
		if (!quiet)
			cout << endl << "Simulation done, angleFitness: " << angleFitness.getFitness(frame) << endl;
		running = false;
		return;
	}
#endif

	 
	frame++;

	if (!quiet)
		cout << "Frame: " << frame << ", AngleReader-Frame: " << behaviourFrame << "\r";


	if (!pause)
	{
		for (int i=0; i<NUM_OF_NAOS; i++)
		{
			// Execute the physics simulation
			if(!worlds[i].step())
			{
				
#if (defined DO_NOT_FALL) && (defined HEADLESS)
				angleFitness.done(frame);
				if (!quiet)
					cout << endl << "Simulation stopped, robot has fallen down, angleFitness: " << angleFitness.getFitness(frame) << endl;
				running = false;
				return;
#endif			
			}
		}
	}

#ifndef HEADLESS
	dsSetColor (0,1,1);
#endif


	// Drawing stuff

#ifndef HEADLESS
#ifdef FLEXIBLE_TEST
	firstBox.draw(0);
	secondBox.draw(0);
#ifndef FLEXBOX
	secondBox.setColor(r, g, b);
	secondBox.draw(0);
	testJoint.setAngle(angle);
#endif
#ifdef FLEXBOX
	firstBox.act((float)parms.sc);
#else
	testJoint.act(Joint::torque);
#endif
	dWorldStep (world,0.002f);
#ifndef FLEXBOX
	RotationMatrix m=secondBox.getRotationMatrix();
	double xang=m.getXAngle();

#endif
#endif
	for (int nao_num=0; nao_num<NUM_OF_NAOS; nao_num++)
		worlds[nao_num].nao.draw(nao_num*1);
#endif


#ifdef LOGGING
	
	for (int nao_num=0; nao_num<NUM_OF_NAOS; nao_num++)
	{
		dReal ori[3], pos[3], lFootPos[3], rFootPos[3];
		dReal lFootOri[3], rFootOri[3];
		worlds[nao_num].nao.getOrientation(ori);
		worlds[nao_num].nao.getPosition(pos);
		worlds[nao_num].nao.getPosition(Nao::footLeft, lFootPos);
		worlds[nao_num].nao.getPosition(Nao::footRight, rFootPos);
		worlds[nao_num].nao.getOrientation(Nao::footLeft, lFootOri);
		worlds[nao_num].nao.getOrientation(Nao::footRight, rFootOri);
		char name[100];
		snprintf(name, 100, "NaoState%i.%s", nao_num, worlds[nao_num].nao.getName().c_str());
		LOG(name, "AngleReaderFrame", behaviourFrame);
		LOG(name, "Pos x", pos[0]);  
		LOG(name, "Pos y", pos[1]);
		LOG(name, "Pos z", pos[2]);
		LOG(name, "Rot x", ori[0]);
		LOG(name, "Rot y", ori[1]);
		LOG(name, "Rot z", ori[2]);
		LOG(name, "Left foot pos x", lFootPos[0]);  
		LOG(name, "Left foot pos y", lFootPos[1]);
		LOG(name, "Left foot pos z", lFootPos[2]);
		LOG(name, "Right foot pos x", rFootPos[0]);  
		LOG(name, "Right foot pos y", rFootPos[1]);
		LOG(name, "Right foot pos z", rFootPos[2]);			
#ifdef ANGLE_INTERPOLATION
		LOG(name, "Ref LHipRoll", interpolatedBehaviour[nao_num][0]*converter[0]);
		LOG(name, "Ref LHipPitch", interpolatedBehaviour[nao_num][1]*converter[1]);
		LOG(name, "Ref LKneePitch", interpolatedBehaviour[nao_num][2]*converter[2]);
		LOG(name, "Ref LAnklePitch", interpolatedBehaviour[nao_num][3]*converter[3]);
		LOG(name, "Ref LAnkleRoll", interpolatedBehaviour[nao_num][4]*converter[4]);
		LOG(name, "Ref RHipRoll", interpolatedBehaviour[nao_num][5]*converter[5]);
		LOG(name, "Ref RHipPitch", interpolatedBehaviour[nao_num][6]*converter[6]);
		LOG(name, "Ref RKneePitch", interpolatedBehaviour[nao_num][7]*converter[7]);
		LOG(name, "Ref RAnklePitch", interpolatedBehaviour[nao_num][8]*converter[8]);
		LOG(name, "Ref RAnkleRoll", interpolatedBehaviour[nao_num][9]*converter[9]);
#else
		LOG(name, "Ref LHipRoll", target[nao_num][0]*converter[0]);
		LOG(name, "Ref LHipPitch", target[nao_num][1]*converter[1]);
		LOG(name, "Ref LKneePitch", target[nao_num][2]*converter[2]);
		LOG(name, "Ref LAnklePitch", target[nao_num][3]*converter[3]);
		LOG(name, "Ref LAnkleRoll", target[nao_num][4]*converter[4]);
		LOG(name, "Ref RHipRoll", target[nao_num][5]*converter[5]);
		LOG(name, "Ref RHipPitch", target[nao_num][6]*converter[6]);
		LOG(name, "Ref RKneePitch", target[nao_num][7]*converter[7]);
		LOG(name, "Ref RAnklePitch", target[nao_num][8]*converter[8]);
		LOG(name, "Ref RAnkleRoll", target[nao_num][9]*converter[9]);

#endif
		LOG(name, "Contact1", worlds[nao_num].hasContact(1));
		LOG(name, "Contact2", worlds[nao_num].hasContact(2));
		LOG(name, "Left foot rot x", lFootOri[0]);
		LOG(name, "Left foot rot y", lFootOri[1]);
		LOG(name, "Left foot rot z", lFootOri[2]);
		LOG(name, "Right foot rot x", rFootOri[0]);
		LOG(name, "Right foot rot y", rFootOri[1]);
		LOG(name, "Right foot rot z", rFootOri[2]);

		worlds[nao_num].nao.log();		

		LOG(name, "Target LHipRoll", interpolatedTarget[nao_num][0]*converter[0]);
		LOG(name, "Target LHipPitch", interpolatedTarget[nao_num][1]*converter[1]);
		LOG(name, "Target LKneePitch", interpolatedTarget[nao_num][2]*converter[2]);
		LOG(name, "Target LAnklePitch", interpolatedTarget[nao_num][3]*converter[3]);
		LOG(name, "Target LAnkleRoll", interpolatedTarget[nao_num][4]*converter[4]);
		LOG(name, "Target RHipRoll", interpolatedTarget[nao_num][5]*converter[5]);
		LOG(name, "Target RHipPitch", interpolatedTarget[nao_num][6]*converter[6]);
		LOG(name, "Target RKneePitch", interpolatedTarget[nao_num][7]*converter[7]);
		LOG(name, "Target RAnklePitch", interpolatedTarget[nao_num][8]*converter[8]);
		LOG(name, "Target RAnkleRoll", interpolatedTarget[nao_num][9]*converter[9]);		
	}
#endif
#ifdef FLEXIBLE_TEST
	LOG("Flextest", "Torque in", testJoint.motor.in);
	LOG("Flextest", "Torque out", testJoint.motor.out);
#endif

#ifdef FLEXBOX
	firstBox.getBox(19)->setColor(0, 0, 0);
#endif
}

#ifndef SOFT_HEADLESS
#ifdef HEADLESS
	void dsDrawBox(const float pos[], const float R[], const float sides[]) { }
	void dsSetColor (float red, float green, float blue) { }
#endif
#endif


// Aufruf: simulator <Config-Pfad>
int main (int argc, char **argv)
{
	if (argc<2)
	{
		cout << "Please specify the config path";
		return -1;
	}
	// setup pointers to drawstuff callback functions
	if (!quiet)
		cout << "Simulations of " << argv[1] << " starts" << endl;

#ifndef HEADLESS
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
 #endif

	// Initialization of the simulation
	initSimulation(argv[1], false);


#ifdef FLEXIBLE_TEST
	dInitODE2(0);
	world = dWorldCreate();			
	space = dHashSpaceCreate (0);			//CollisionCheck-Welt erzeugen; 0 - SpaceID
	contactgroup = dJointGroupCreate (0);		//ContactJointGroup erzeugen; 0 - SpaceID
	dWorldSetGravity (world,0,0,-9.81f);		//Gravitation setzen
	ground = dCreatePlane (space,0,0,1,0);	//Grundebene erzeugen; space - Raum, Gleichung fuer die Ebene: 0x+0y+1z=0
	dReal axis[]={1, 0, 0};
	dReal anchor[]={0, 0, 0.5};
	dReal bodyPos[]={0, 0, 0.25};
	dReal upperBodyPos[]={0, 0, 0.75};
	dReal bodySize[]={0.05f, 0.05f, 0.5f};
	dReal zAxis[]={0, 0, 1};
#ifdef FLEXBOX
	secondBox.createPhysics(world, space, bodyPos, bodySize, 100);

	firstBox.createPhysics(world, space, upperBodyPos, bodySize, zAxis, 1, 20);
	dJointID id=dJointCreateFixed(world, 0);
	dJointAttach(id, firstBox.getBox(0)->getID(), secondBox.getID());
	dJointSetFixed(id);
	id=dJointCreateFixed(world, 0);
	dJointAttach(id, secondBox.getID(), 0);
	dJointSetFixed(id);

#else
	firstBox.createPhysics(world, space, bodyPos, bodySize, 100);
	dJointID id=dJointCreateFixed(world, 0);
	dJointAttach(id, firstBox.getID(), 0);
	dJointSetFixed(id);
	bodyPos[2]+=0.5;
	secondBox.createPhysics(world, space, bodyPos, bodySize, 100); // 0.1
	dMatrix3 R;
	dRFromAxisAndAngle(R, 1, 0, 0, 0);
	dBodySetRotation(secondBox.getID(), R);
	testJoint.createPhysics(
		world,
		firstBox,
		secondBox,
		anchor,
		axis,
		1,
		1);
#endif

	PIDController::PIDParams pidParams;
	pidParams.p=(float)parms.p;
	pidParams.i=(float)parms.i;
	pidParams.d=(float)parms.d;
	pidParams.frameLen=0.02f;
	testJoint.setPIDParams(pidParams, Joint::torque);
#endif

	// Execute the simulation loop
#ifndef HEADLESS
	dsSimulationLoop (argc,argv,1024,768,&fn);
#else
	while(running)
	  simLoop(false);
#endif

	quitSimulation();

	return 0;
}



