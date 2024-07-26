#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include "csvclass/csvparser.h"

class Behaviour
{
public:
	Behaviour();
	Behaviour(string path);
	~Behaviour();
	void getAngles(float *angles, int numOfAngles);
private:
	ifstream infile;
	CSVParser parser;
};