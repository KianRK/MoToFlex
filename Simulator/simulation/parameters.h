#pragma once
#include <stdio.h>
#include <string>
#include "config.h"
#include <eaconfig.h>

using namespace std;

#define NUM_OF_PARAMS 8+NUM_OF_GEARTYPES*7

class Parameters : Config
{
public:
	Parameters(void);
	~Parameters(void);

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
