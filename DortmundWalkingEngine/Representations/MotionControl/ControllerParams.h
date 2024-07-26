/**
* @file ControllerParams 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class ControllerParams
 *
 * Contains the parameters for the ZMP/IP-Controller calculated by the
 * matlab script writeParams.m
 */
class ControllerParams : public Streamable
{

public :
	double dt; /**< Duration of one frame */
	double z_h;	/**< desired CoM position (height) */
	int N; /**< Length of the preview phase */
	double *Gd; /**< Data for preview controller */
	double A0[3][3]; /**< Data for preview controller */
	double Gi; /**< Data for preview controller */
	double Gx[3]; /**< Data for preview controller */
	double b0[3]; /**< Data for preview controller */
	double c0[3]; /**< Data for preview controller */
	double L[3][2]; /**< Data for preview controller */

	/** Constructor */
	ControllerParams(){};

	/** Destructor */
	~ControllerParams()
	{};

	void serialize(In* in,Out* out) {};
};
