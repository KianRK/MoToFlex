#include "behaviour.h"

Behaviour::Behaviour()
{
	string sLine;
	infile.open("ExternalSimulator.csv");
	getline(infile, sLine); // Erste Zeile überspringen
}

Behaviour::Behaviour(string path)
{
	string sLine;
	infile.open(path.c_str());
	getline(infile, sLine); // Erste Zeile überspringen
}


Behaviour::~Behaviour()
{
	infile.close();
}

void Behaviour::getAngles(float *angles, int numOfAngles)
{
	string sLine;
	short converter[]={1, -1, -1, -1, 1, -1, -1, -1, -1, -1};

	do
	{
		if (infile.eof())
		{
			infile.seekg(ios_base::beg);
			getline(infile, sLine); // Erste Zeile überspringen
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