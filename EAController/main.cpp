
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
#include <iostream>
#include "jobcontainer.h"
#include "generation.h"
#include "eaparams.h"

#include "compat.h"
#include <csvlogger.h>

using namespace std;

// argv[1] is path to configuration file

EAParams eaParams;


int main (int argc, char **argv)
{
	float minFitness=-1;
	Generation::IndividualList elite;
					  
	cout << endl << "Loading params..." << endl;

	eaParams.load(argv[1]);
	CSVLogger::basepath=eaParams.pathToShare;

	cout << "Initializing first generation.." << endl;

	EAController::Individual::initRandom();
	Generation *generation=NULL;
	int currentGeneration=1;
	Generation *nextGeneration=new Generation(currentGeneration);
	bool cont=true;


	while (cont)
	{
		elite.clear();
		if (generation!=NULL)
			delete generation;
		generation=nextGeneration;
		nextGeneration=NULL;
		cout << "Starting jobs of generation " << generation->getGenerationNumber() << endl;
		generation->startEvaluation();

		cout << "Waiting for all jobs ..." << endl;
		
		while (!generation->checkForFinished())
			ws_sleep(5000);
		
		cout << endl << "Jobs done, " << generation->getFailed() << " failed, searching for parents... ";

		nextGeneration=new Generation(++currentGeneration);
		int betterChilds=generation->naturalSelection(elite);

		generation->log(elite);
		if (minFitness==-1 || minFitness>elite[0]->getFitness())
		{
			ws_copy(elite[0]->getGenomPath(), eaParams.pathToShare+"BestOf.gnm");
			minFitness=(float)elite[0]->getFitness();
		}

		cout << "found:	";
		for (Generation::IndividualIterator it=elite.begin(); it!=elite.end(); it++)
			cout << (*it)->generateID() << ", ";
		cout << endl;

		cout << "Initializing next generation..." << endl;
		nextGeneration->nextGeneration(elite, betterChilds);

		if (generation->getGenerationNumber()>eaParams.maxGenerations)
			cont=false;
	}

	if (generation!=NULL)
		delete generation;
	if (nextGeneration!=NULL)
		delete nextGeneration;

	return 0;
}
