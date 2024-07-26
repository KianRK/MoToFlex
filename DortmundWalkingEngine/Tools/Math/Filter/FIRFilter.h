#pragma once

#include "../../DynamicRingBuffer.h"
#include "Filter.h"
#include <string>
#include <vector>

using namespace std;

class FIRFilter :
	public Filter
{
public:
	FIRFilter(void);
	~FIRFilter(void) 
	{
		delete buffer;
	};

	double nextValue(double v);

	bool readCoefficients(string path);
	void setCoefficients(double *coefficients, int n);

private:
	unsigned int n;
	DynamicRingBuffer<double> *buffer;
	vector<double> coefficients;
};
