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
#include "jobcontainer.h"
#include "compat.h"
#include <iostream>
#include <sstream>
#include <string.h>

using namespace std;

#ifdef TRACING
#include <iostream>
#endif
JobContainer::JobContainer(void)
{
	next=0;
}

JobContainer::~JobContainer(void)
{
}

int JobContainer::add(string cmd)
{
	jobs[next%NUM_OF_NODES].push_back(cmd);
	next++;
	return (next-1)%NUM_OF_NODES;
}

void JobContainer::exec()
{
	next=0;
	for (int i=0; i<NUM_OF_NODES; i++)
	{
		if (jobs[i].size()>0)
		{
			exec(i);
			jobs[i].clear();
		}
	}
}

void JobContainer::exec(int job)
{
	FILE *f=fopen("/tmp/torquescript", "w");
	fprintf(f, "#!/bin/bash\n"); 
	fprintf(f, "#PBS -l nodes=1\n");
	fprintf(f, "#PBS -l mem=100mb\n");
	fprintf(f, "#PBS -l walltime=00:05:00\n");
	fprintf(f, "#PBS -l cput=1000\n");
#ifdef QUIET
	fprintf(f, "#PBS -e /dev/null\n");
	fprintf(f, "#PBS -o /dev/null\n");
#endif
	for (JobIterator it=jobs[job].begin(); it!=jobs[job].end(); it++)
		fprintf(f, "%s\n", (*it).c_str());	
	fclose(f);

	id[job]=ws_execute("qsub /tmp/torquescript");
#ifdef TRACING
	cout << id[job] << endl;
#endif
}

bool JobContainer::isFinished(int jobID)
{
	string status=getStatus(jobID);
	const char *c_status=status.c_str();
	if (strncmp(c_status, "end of output", 13)!=0)
	{
#ifdef TRACING
		cout << "isFinished(" << jobID << "): true, Status: " << c_status << endl;
#endif
		return true;
	}
	else
	{
#ifdef TRACING
	cout << "isFinished(" << jobID << "): false" << endl;
#endif
		return false;
	}
}

int JobContainer::exitCode(int jobID)
{
	std::istringstream str(getStatus(jobID));
	int code=0;
	str >> code;
	
#ifdef TRACING
	cout << "exit(" << jobID << "): " << code << endl;
#endif

	return code;
}

string JobContainer::getStatus(int jobID)
{
	string cmd="tracejob -q ";
	cmd+=id[jobID]+" | awk 'BEGIN{regex=\"Exit_status=[-]*[0-9]*\"}{match($0, regex);if (RLENGTH!=-1){print substr($0,RSTART+12,RLENGTH-12);exit;}}END{print \"end of output\";}'";
#ifdef TRACING
	cout << "getStatus(" << jobID << "), executing: " << cmd << endl;
#endif
	return ws_execute(cmd);
}