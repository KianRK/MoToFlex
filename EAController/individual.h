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

#include "paths.h"
#include <genom.h>
#include <string>
#include "eaparams.h"
#include "config.h"
#include "strategyinformation.h"
#include "jobcontainer.h"
#include "Rng/GlobalRng.h"


#include <time.h>			
#ifdef WIN32
#include <random>
#else
#include <tr1/random>
#endif
#undef min
#undef max

using namespace std;
class Generation;

typedef std::tr1::mt19937 EngineType;
typedef std::tr1::normal_distribution<double> DistributionType;
					  
// Returns a random number between [-1, 1)

inline float UnitRV() {
  return 2 * ((float) rand()/RAND_MAX) - 1;
}   

// Uses the Polar method to generate a normal distribution with mean 0 and standard deviation 1
// http://ct.radiology.uiowa.edu/~jiang...ml/node41.html
inline float GaussianValue(int mean, int range) {
  static int iset = 0;
  static float gset;
  float fac, rsq, v1, v2;
  float result;

  
  if  (iset == 0)  {
    do {
      v1  = (float) (2.0 * UnitRV() - 1.0);
      v2  = (float) (2.0 * UnitRV() - 1.0);
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac = (float) (sqrt (-2.0 * log (rsq) / rsq));
    gset = v1 * fac;
    iset = 1;
    result = v2 * fac;
  } 
  else {
    iset = 0;
    result = gset;
  }
  
  return mean+range*result;
}

namespace EAController
{

	class Individual : Config
	{
	public:
		enum State {unstarted, running, finished, failed};

		Individual(int generation,
			int individual,
			string initialValuesPath,
			string variancesPath);

		~Individual(void);

		// reuse the object for an individual of the next generation
		void recycle(int generation, int individual);

		// Writes the genome and config file. Filename will be path
		// plus generation and individual number
		void write(Paths &config);

		// returns the full filename (without path) the path configuration
		string getFilename();

		// Enqueues a job for fitness evaluation
		void startFitnessEvaluation(Paths &paths, JobContainer &jobc);

		// Check, if the simulation stopped
		// If yes, set fitness
		bool isEvaluationFinished(JobContainer &jobc);

		double getFitness();

		// needed for sort()
		bool operator<(Individual& a)
		{
			return getFitness()<a.getFitness();	
		}

		static void initRandom()
		{
			eng.seed((unsigned int)time(NULL));
		}

		static double getNormalDistribution(double mean, double variance)
		{
			eng();
			// This is NOT N(mean, sigma^2), the params
			// are the plain mean and sigma
			// so dist(a, b) is N(a, b^2) distributed
			DistributionType dist(mean, variance);
			std::tr1::variate_generator<EngineType, DistributionType> gVariate( eng, dist );
			return gVariate();
			//return Rng::gauss(mean, variance);
			//return GaussianValue(mean, variance);

		}

		static double getLogNormalDistribution(double mean, double variance)
		{
			return exp(getNormalDistribution(mean, variance));
		}

		// changed every new generation to N(0,1)
		static double globalRandomNumber;

		void updateStrategyParams(StrategyInformation *strinf) {};

		int getIndividualNum() { return individual; }
		int getGenerationNum() { return generation; }
													 
		string generateID() const;
		string getGenomPath() { return genomPath; }

		void cleanup();

		int age;
		Genom genom, variances;
		State state;
		
	protected:
		EAParams::JobType jobType;
		int generation, individual, checks;
		static EngineType eng;
		string fitnessPath, tmpFile, genomPath, configPath, batchFile;
#ifdef EA_TEST
		double fitness;
#endif
		int jobID;

		unsigned long getPrntNum(int numOfParents)
		{
			return ((unsigned long long)eng()*((unsigned long long)numOfParents))/eng.max();
		}
	};

}