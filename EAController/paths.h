
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
#include <string>
#include <config.h>
#include "eaconfig.h"
using namespace std;

class Paths : Config
{
public:
	Paths(void);
	~Paths(void);

	void setBasePath(string path);
	void setSubDir(string subDir) { subdir=subDir; }

	// These functions sets the file names, use
	// it without providing a path or a suffix
	void setParameterFn(string fn);
	void setBehaviourFn(int naoNum, string fn);
	void setLogFn(string fn);
	void setFitnessFn(string fn);
	void setTarget(int naoNum, string fn);

	void write(string fn);

	string getFullParameterPath() { return path+subdir+parmFn; }
	string getFullBehaviourPath(int naoNum) { return path+behFn[naoNum]; }
	string getFullLogPath() { return path+subdir+logFn; }
	string getFullFitnessPath() { return path+subdir+FitFn; }
	string getFullTargetPath(int naoNum) { return path+tgtFn[naoNum]; }
	string getFullTempPath(string tmp) { return path+subdir+tmp+".tmp"; }
	string getFullShPath(string sh) { return path+subdir+sh+".sh"; }
	string getFullPath(string fn) { return path+fn; }
	string getFullCfgPath(string fn) { return path+subdir+fn+".cfg"; }
private:
	string subdir;
	string path, parmFn, 
		behFn[NUM_OF_NAOS+1], 
		tgtFn[NUM_OF_NAOS+1],
		logFn, FitFn;
};
