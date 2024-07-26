#include "cmaindividual.h"
#include "eaparams.h"
#include "eaconfig.h"

#ifdef TRACING
#include <iostream>
using namespace std;
#endif

extern EAParams eaParams;

CMAIndividual::CMAIndividual(int generation,
		int individual,
		string initialValuesPath,
		string variancesPath)
		: EAController::Individual(generation,
		individual,
		initialValuesPath,
		variancesPath)
{
	IndividualCT<ChromosomeCMA> newind(1);
#ifdef TRACING
	cout << generateID() 
		 << ": init("
		 << getDimension()
		 << ", "
		 << eaParams.variance
		 << ", 0, 1, "
		 << eaParams.generationSize
		 << endl;
#endif
	newind[0].init(getDimension(), eaParams.variance, 0, 1, eaParams.generationSize);
	ind=newind;
	copyToLib(); 
}

void CMAIndividual::mutate(MutationInformation *mutInf)
{
#ifdef TRACING
	cout << generateID() << ": mutate()" << endl;
#endif
	ind[0].mutate();
	copyToEAController();
}


void CMAIndividual::combine(IndividualList &parents)
{
#ifdef TRACING
	cout << generateID() << ": combine(), usedParent set" << endl;
#endif
	int s=getPrntNum(parents.size());
	ind=parents[s]->ind;
	usedParent=ind;
	copyToEAController();
}

void CMAIndividual::updateStrategyParams(StrategyInformation *strinf)
{
#ifdef TRACING
	cout << generateID() << ": updateGlobalStepsize(" << strinf->numOfBetterChilds << ")" << endl;
#endif
	if (strinf->numOfBetterChilds==-1)
		return;
	ind[0].updateGlobalStepsize(strinf->numOfBetterChilds);
	if (strinf->numOfBetterChilds && usedParent.size()!=0)
	{
#ifdef TRACING
	cout << generateID() << ": updateCovariance(" << usedParent << ")" << endl;
#endif
		ind[0].updateCovariance(usedParent);
	}
}

void CMAIndividual::copyToLib()
{
	for (int i=0; i<getDimension(); i++)
	{
		ind[0][i]=genom.chromosome[i];
	}
}

void CMAIndividual::copyToEAController()
{
	for (int i=0; i<getDimension(); i++)
	{
		genom.chromosome[i]=ind[0][i];	
	}
}
