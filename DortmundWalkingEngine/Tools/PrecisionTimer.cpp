#include "PrecisionTimer.h"

#ifdef _WIN32
PrecisionTimer::PrecisionTimer() {
  QueryPerformanceFrequency(&freq);
}

void PrecisionTimer::tstart(size_t measurementIndex) {
	QueryPerformanceCounter(&_tstart[measurementIndex]);
}

void PrecisionTimer::tend(size_t measurementIndex){
  QueryPerformanceCounter(&_tend[measurementIndex]);
}

double PrecisionTimer::tval(size_t measurementIndex){ 
	return ((double)_tend[measurementIndex].QuadPart -
		(double)_tstart[measurementIndex].QuadPart)/((double)freq.QuadPart);
}
#else 
PrecisionTimer::PrecisionTimer() {

}

void PrecisionTimer::tstart(size_t measurementIndex){
	gettimeofday(&_tstart[measurementIndex], &tz);
}

void PrecisionTimer::tend(size_t measurementIndex){
	gettimeofday(&_tend[measurementIndex],&tz);
}

double PrecisionTimer::tval(size_t measurementIndex){
	double t1, t2;
	t1 =  (double)_tstart[measurementIndex].tv_sec + (double)_tstart[measurementIndex].tv_usec/(1000*1000);
	t2 =  (double)_tend[measurementIndex].tv_sec + (double)_tend[measurementIndex].tv_usec/(1000*1000);
	return t2-t1;
}

#endif
