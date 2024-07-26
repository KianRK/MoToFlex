#include "parameters.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;

Parameters::Parameters(void)
{
	torqueFilterLen=1;
}

Parameters::~Parameters(void)
{
}

void Parameters::load(string path)
{
	open(path.c_str());

	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		jointFriction[k]=readDouble();
	
	dynamicFriction=readDouble();
	flexibleFriction=readDouble();
	fMax=readDouble();
	p=readDouble();
	i=readDouble();
	d=readDouble();
	sc=readDouble();
	mu=readDouble();
	slip1=readDouble();
	slip2=readDouble();
	soft_erp=readDouble();
	soft_cfm=readDouble();
	erp=readDouble();
	cfm=readDouble();
	L=readDouble();
	R=readDouble();
	maxVolt=readDouble();
	K=readDouble();
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		B[k]=readDouble();

	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		tolerance[k]=readDouble();

	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		mass[k]=readDouble();

	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		gearSc[k]=readDouble();

	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		length[k]=readDouble();

	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		innerDamping[k]=readDouble();

	stiffness=readDouble();	
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		reduction[k]=readDouble();



	frameLen=(float)readDouble();
	controllerFreq=(int)readDouble();
	numOfSubBoxes=(int)readDouble();
	measureStart=(int)readDouble();
	measureStop=(int)readDouble();
	close();
}

void Parameters::write(string path)
{
	open(path.c_str(), true);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("jointFriction", jointFriction[k]);
	writeDouble("dynamicFriction", dynamicFriction);
	writeDouble("flexibleFriction", flexibleFriction);
	writeDouble("fMax", fMax);
	writeDouble("p", p);
	writeDouble("i", i);
	writeDouble("d", d);
	writeDouble("sc", sc);
	writeDouble("mu", mu);
	writeDouble("slip1", slip1);
	writeDouble("slip2", slip2);
	writeDouble("soft_erp", soft_erp);
	writeDouble("soft_cfm", soft_cfm);
	writeDouble("erp", erp);
	writeDouble("cfm", cfm);
	writeDouble("L", L);
	writeDouble("R", R);
	writeDouble("maxVolt", maxVolt);
	writeDouble("K", K);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("B", B[k]);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("ToleranceDelay", tolerance[k]);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("Mass", mass[k]);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("GearSc", gearSc[k]);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("HullLength", length[k]);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("InnerDamping", innerDamping[k]);
	writeDouble("stiffness", stiffness);
	for (int k=0; k<NUM_OF_GEARTYPES; k++)
		writeDouble("ReductionType", reduction[k]);
	writeDouble("frameLen", frameLen);
	writeDouble("controllerFreq", controllerFreq);
	writeDouble("numOfSubBoxes", numOfSubBoxes);
	writeDouble("measureStart", measureStart);
	writeDouble("measureStop", measureStop);
	close();
}

