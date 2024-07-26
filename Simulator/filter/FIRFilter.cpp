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
#include "FIRFilter.h"
#include <fstream>
#include <stdlib.h>

FIRFilter::FIRFilter(void)
{
	static double d=1;
	buffer=0;
	setCoefficients(&d, 1);
}

double FIRFilter::nextValue(double v)
{
	buffer->add(v);
	double output=0;
	for (int i=0; i<buffer->getNumberOfEntries(); i++)
		output+=buffer->getEntry(i)*coefficients[i];
	return output;
}

void FIRFilter::setCoefficients(double *coefficients, int n)
{
	this->n=n;

	if (buffer!=0)
		delete buffer;
	
	buffer=new DynamicRingBuffer<double>(n);
	this->coefficients.clear();

	for (int i=0; i<n; i++)
		this->coefficients.push_back(coefficients[i]);
}

bool FIRFilter::readCoefficients(string path)
{
	bool running=false;
	string line;
	char *stopstring;

	if (buffer!=0)
		delete buffer;
	coefficients.clear();

	ifstream fcf(path.c_str());
	if (fcf.is_open())
	{
		while (!fcf.eof())
		{
			getline(fcf, line);
			if (line[0]=='%')
				continue;
			if (line.find("Numerator:", 0)!=string::npos)
			{
				running=true;
				continue;
			}

			double d=strtod(line.c_str(), &stopstring);

			if (d==0)
				running=false;
			
			if (running)
				coefficients.push_back(d);
			
		}
		fcf.close();
	}

	n=coefficients.size();

	buffer=new DynamicRingBuffer<double>(n);

	return true;
}
