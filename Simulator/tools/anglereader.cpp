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
#include "anglereader.h"
#include <stdlib.h>
#include "eaconfig.h"

AngleReader::AngleReader()
{
	string sLine;
	infile.open("ExternalSimulator.csv");
	getline(infile, sLine); // Erste Zeile �berspringen
}

AngleReader::AngleReader(string path)
{
	string sLine;
	infile.open(path.c_str());
	if (!infile.is_open())
	{
		cout << "Could not open angle file: " << path << endl;
		exit(1);
	}
	getline(infile, sLine); // Erste Zeile �berspringen
}


AngleReader::~AngleReader()
{
	infile.close();
}

void AngleReader::getAngles(float *angles, int numOfAngles)
{
	string sLine;
	
	do
	{
		if (infile.eof())
		{
			infile.seekg(ios_base::beg);
			getline(infile, sLine); // Erste Zeile �berspringen
		}
		getline(infile, sLine);
	}
	while(sLine=="");

	parser << sLine;
	double a;
	for (int i=0; i<numOfAngles; i++)
	{
		parser >> a;
		angles[i]=float(a)*converter[i];
	}
}
