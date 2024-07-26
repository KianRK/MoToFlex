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
#include "generation.h"
#include <algorithm>
#include "eaconfig.h"
//#define LOGGING
#include <csvlogger.h>
#include <iostream>
#include "strategyinformation.h"

extern EAParams eaParams;

bool Sort(EAController::Individual* d1, EAController::Individual* d2)
{
	return *d1 < *d2;
}

Generation::Generation(int generation)
{
	this->generation=generation;
	config.setBasePath(eaParams.pathToShare);
	config.setSubDir(eaParams.subDir);
	for (int i=0; i<NUM_OF_NAOS; i++)
		config.setBehaviourFn(i, eaParams.behFn[i]);
	for (int i=0; i<NUM_OF_NAOS; i++)
		config.setTarget(i, eaParams.targetFn[i]);
	for (int i=0; i<eaParams.generationSize; i++)
	{
		individuals.push_back(new IndividualType(generation,
		i,
		eaParams.pathToShare+eaParams.initialValuesFn,
		eaParams.pathToShare+eaParams.variancesFn));
		//individuals[i]->mutate(&mutationInformation);
	}
	highestFitnes=numeric_limits<double>::infinity();
}

Generation::~Generation(void)
{
	for (int i=0; i<eaParams.generationSize; i++)
	{
		IndividualType *in=(IndividualType *)individuals.back();


#ifdef CLEANUP
		// Delete all files of dead idividuals (note: parents
		// are not dead)
		if (i<eaParams.generationSize-eaParams.numOfParents)
			in->cleanup();
#endif

		individuals.pop_back();
		delete in;
	}	
}

int Generation::naturalSelection(IndividualList &elite)
{
	// count better children
	int newParents=0;
	for (IndividualIterator it=individuals.begin(); it!=individuals.end(); it++)
	{
		if ((*it)->getFitness()<highestFitnes)
			newParents++;
	}

	std::sort(individuals.begin(), individuals.end(), Sort);
	IndividualIterator it=elite.begin();
	elite.insert(it, individuals.begin(), individuals.begin()+eaParams.numOfParents);

	return newParents;
}

bool Generation::checkForFinished()
{
   	for (IndividualIterator it=individuals.begin(); it!=individuals.end(); it++)
	{
		if (!(*it)->isEvaluationFinished(jobc))
			return false;
	}
	return true;
}

void Generation::startEvaluation()
{
	for (IndividualIterator it=individuals.begin(); it!=individuals.end(); it++)
	{
#ifndef EA_TEST
		(*it)->write(config);
#endif
		(*it)->startFitnessEvaluation(config, jobc);
	}
	jobc.exec();
}

int Generation::getFailed()
{
	int failed=0;
	for (IndividualIterator it=individuals.begin(); it!=individuals.end(); it++)
	{
		if ((*it)->state==EAController::Individual::failed)
			failed++;
	}
	return failed;
}

JobContainer Generation::jobc;

void Generation::log(IndividualList &elite)
{
	LOG("BestOf", "Generation", elite[0]->getGenerationNum());
	LOG("BestOf", "Individual", elite[0]->getIndividualNum());
	LOG("BestOf", "Fitness", elite[0]->getFitness());
	LOG("BestOf", "Mutationsschrittweite 0", elite[0]->variances.chromosome[0]);
	LOG("BestOf", "Wert 0", elite[0]->genom.chromosome[0]);

	for (IndividualIterator it=individuals.begin(); it!=individuals.end(); it++)
	{
		LOG("Output", "Generation", (*it)->getGenerationNum());
		LOG("Output", "Individual", (*it)->getIndividualNum());
		if ((*it)->getFitness()==std::numeric_limits<float>::infinity())
			LOG("Output", "Fitness", -1);
		else
			LOG("Output", "Fitness", (*it)->getFitness());
	}
}

void Generation::nextGeneration(IndividualList &parents, int betterChilds)
{
	StrategyInformation strinf;
	strinf.numOfBetterChilds=betterChilds;
	// copy parents and let them age, if they are too old,
	// let them die (do not copy, but still use them  as parents)
	int copied=0;
	for (int i=0; i<eaParams.numOfParents; i++)
	{
		parents[i]->age++;
		parents[i]->updateStrategyParams(&strinf);
		if (parents[i]->age<eaParams.maxAge)
		{
			*individuals[copied]=*parents[i];			
			copied++;
		}
	}

	highestFitnes=(*parents[eaParams.numOfParents-1]).getFitness();	

	// now choose parents for each new individual
	// mutate the children
	for (int i=copied; i<eaParams.generationSize; i++)
	{
		IndividualType::globalRandomNumber=IndividualType::getNormalDistribution(0,1);
		individuals[i]->combine(parents);
		mutationInformation.collect(*this);
		individuals[i]->mutate(&mutationInformation);
	}

}

