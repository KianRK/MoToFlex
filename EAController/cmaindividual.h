#pragma once
#include <individual.h>
#include <string>
#include <vector>
#include "mutationinformation.h"
#include <EALib/IndividualT.h>
#include <EALib/ChromosomeCMA.h>

using namespace std;

class CMAIndividual :
	public EAController::Individual
{
public:
	typedef std::vector<CMAIndividual *> IndividualList;
	typedef IndividualList::iterator IndividualIterator;
	CMAIndividual(int generation,
		int individual,
		string initialValuesPath,
		string variancesPath);

	~CMAIndividual(void) {};

	void mutate(MutationInformation *mutInf=NULL);
	void combine(IndividualList &parents);

	// only used if this individual becomes a parent
	void updateStrategyParams(StrategyInformation *strinf);

private:
	IndividualCT<ChromosomeCMA> ind, usedParent;

	void copyToLib();
	void copyToEAController();
	int getDimension() { return NUM_OF_PARAMS; };
};
