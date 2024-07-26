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
#include <stdio.h>
#include <string>
#include "config.h"
#include <eaconfig.h>

using namespace std;

#define NUM_OF_PARAMS 8+NUM_OF_GEARTYPES*7

class Genom : Config
{
public:
	Genom(void);
	~Genom(void);

	void load(string path);
	void write(string path);

	union
	{
		struct
		{

			// simulation parameters
			// all these parameters are subjected to mutate 
			double sc,		// spring consta	n
				B[NUM_OF_GEARTYPES],			// constant for speed dependent gear friction
				soft_cfm,			
				cfm,
				p,
				i,
				d,
				jointFriction[NUM_OF_GEARTYPES],
				flexibleFriction,

				tolerance[NUM_OF_GEARTYPES],   // if SIMPLE_TOLERANCE_GEARS this is the delay
								// time, if FLEXIBLE_TOLERANCE_GEARS this
								// is the length of the hull	
				
				mass[NUM_OF_GEARTYPES],		 // the mass of the gears before the point
								 // of tolerance and flexibility
				gearSc[NUM_OF_GEARTYPES],		 // spring constant of the gears
				length[NUM_OF_GEARTYPES],		 // length of the mass hull
				innerDamping[NUM_OF_GEARTYPES],

				dynamicFriction; // only used for static and dynamic
									// friction model

		};

		// this chromosome is for the cma. Change its size to
		// change the number of params to mutate.
		double chromosome[NUM_OF_PARAMS];
	};


	// other parameters, no mutation

	double	maxVolt,
			stiffness,
			reduction[NUM_OF_GEARTYPES],
			soft_erp,
			erp,
			mu,
			L,			// motor inductance
			R,			// and resistance
			slip1,
			slip2,	
			fMax,
			K;

	float frameLen;
	
	int measureStart, 
		measureStop, 
		numOfSubBoxes,
		torqueFilterLen,
		controllerFreq;
};
