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
#include "individual.h"
#include "compat.h"
#include <iostream>

#define QUIET

extern EAParams eaParams;

using namespace std;
using namespace EAController;

Individual::Individual(int generation,
					   int individual,
					   string initialValuesPath,
					   string variancesPath)
{
	recycle(generation, individual);
	age=0;
	genom.load(initialValuesPath);
	variances.load(variancesPath);
	state=unstarted;
}

Individual::~Individual(void)
{
}

EngineType Individual::eng;
double Individual::globalRandomNumber;
void Individual::recycle(int generation, int individual)
{
	this->generation=generation;
	this->individual=individual;
}

string Individual::generateID() const
{
	char str[1000];
	sprintf(str, "G%i-I%i", generation, individual);
	string str2=str;
	return str2;
}

void Individual::write(Paths &config)
{
	config.setLogFn(generateID());
	config.setParameterFn(generateID());
	config.setFitnessFn(generateID());
	config.write(getFilename());
	genomPath=config.getFullParameterPath();
	genom.write(genomPath);
	configPath=config.getFullCfgPath(generateID());
	fitnessPath=config.getFullFitnessPath();
}

string Individual::getFilename()
{
	string fn=generateID();
	return fn;
}
void Individual::startFitnessEvaluation(Paths &paths, JobContainer &jobc)
{
#ifndef EA_TEST
	jobType=eaParams.jobType;
	if (state!=unstarted)
		return;

	state=running;
	if (jobType==EAParams::local)
	{
		string command=eaParams.pathToBin+eaParams.simulatorName;
		command+=" ";
		command+=paths.getFullCfgPath(getFilename());
		if (jobType==EAParams::local)
			ws_execute(command);			
	}
	if (jobType==EAParams::cluster)
	{
		if (OS==1)
		{
			checks=0;

			string command=eaParams.pathToBin+eaParams.simulatorName+" ";
			command+=paths.getFullCfgPath(getFilename());

			jobID=jobc.add(command);

		}
		if (OS==0)
		{
			printf("windows cluster not supported");
			exit(2);
		}
	}
#else
	fitness=0;	
	const double offset = 0;
	for (int i=0; i<sizeof(genom.chromosome)/sizeof(double); i++)
	{
		if (variances.chromosome[i]!=0)
			fitness+=(genom.chromosome[i]-offset)*(genom.chromosome[i]-offset);
	}
#endif
}

void Individual::cleanup()
{
	ws_delete(fitnessPath);
	ws_delete(tmpFile);
	ws_delete(genomPath);
	ws_delete(configPath);
	ws_delete(batchFile);
}

bool Individual::isEvaluationFinished(JobContainer &jobc)
{
#ifndef EA_TEST
	if (state==finished || state==failed)
		return true;

	if (jobType==EAParams::local)
	{
		state=finished;
		return true;
	}
	
	if (checks>5)
	{
		cout << generateID() << ": timeout!\t";
		state=failed;
		return true;
	}
	checks++;

	if (ws_exists(fitnessPath))
	{
		state=finished;
		cout << generateID() << ": " << getFitness() << "\t";
		return true;
	}
	else
	{
		if (jobc.isFinished(jobID))
		{
			state=failed;
			cout << generateID() << ": failed (" << jobc.exitCode(jobID) << ")!\t";
			return true;
		}
	}
	return false;
#else
	return true;
#endif
}

double Individual::getFitness()
{
#ifndef EA_TEST
	if (state!=finished || state==failed)
		return std::numeric_limits<float>::infinity();
	double fitness;
	open(fitnessPath.c_str(), false);
	fitness=readDouble();
	close();
	return fitness;
#else
	return this->fitness;
#endif
}
