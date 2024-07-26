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
#include <vector>
#include <string>
#include "eaconfig.h"

using namespace std;

class JobContainer
{
public:
	JobContainer(void);
	~JobContainer(void);

	int add(string cmd);
	void exec();
	bool isFinished(int jobID);
	int exitCode(int jobID);

private:
	typedef std::vector<string> JobList;
	typedef JobList::iterator JobIterator;

	JobList jobs[NUM_OF_NODES];
	string id[NUM_OF_NODES];

	int next;

	// execute a single job
	void exec(int job);
	string getStatus(int jobID);
};
