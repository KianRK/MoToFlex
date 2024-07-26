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
#include "simpleindividual.h"
#include "detparindividual.h"
#include "selfadaptindividual.h"
#include "selfadaptindividualnorecomb.h"
#include <vector>
#include "eaparams.h"
#include "mutationinformation.h"
#include "cmaindividual.h"
#include "jobcontainer.h"
		
#include <time.h>			
#ifdef WIN32
#include <random>
#else
#include <tr1/random>
#endif
#undef min
#undef max

// set the individual type here
typedef SelfAdaptIndividualNoRecomb IndividualType;
//typedef CMAIndividual IndividualType;
				 

class Generation
{
	friend class MutationInformation;
public:
	typedef IndividualType::IndividualList IndividualList;
	typedef IndividualList::iterator IndividualIterator;

public:
	Generation(int generation);
	~Generation(void);

	// creates the next generation (group ... ;-) )
	// increments the generation counter
	void nextGeneration(IndividualList &parents, int betterChilds);

	// Writes config files, start fitness evaluation jobs
	void startEvaluation();

	// returns true, if all jobs finished
	bool checkForFinished();

	// stores the individuals (pointers) with the highest fitness in elite
	int naturalSelection(IndividualList &elite);

	int getGenerationNumber() { return generation; }

	void log(IndividualList &elite);

	int getFailed();
private:
	int generation, numOfDone;
	IndividualList individuals;
	Paths config;
	MutationInformation mutationInformation;
	static JobContainer jobc;
	double highestFitnes;
};
