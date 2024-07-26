/**
* @class WalkingEngineParams 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
* @author <a href="mailto:florian.wilmshoever@tu-dortmund.de">Florian Wilmshöver</a>
*/
#ifndef _WALKINGENGINEPARAMS_H
#define _WALKINGENGINEPARAMS_H
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

/**
* @class WalkingEngineParams 
* Contains the parameters of the walking engine. See the "Nao Devils Team Report 2010" for a detailed description.
*/
class WalkingEngineParams : public Streamable
{


public :

	float footPitch;
	float maxWalkPitch;
	float pitchSpeed;
	float xOffset;
	float stepHeight;
	float sensorControlRatio;
	float doubleSupportRatio;
	int crouchingDownPhaseLength;
	int startingPhaseLength;
	int stoppingPhaseLength;
	float armFactor;
	float arms1;
	int zmpSmoothPhase;
	int maxSpeedXForward;
	int maxSpeedXForwardOmni;
	int maxSpeedXBack;
	int maxSpeedYLeft;
	int maxSpeedYRight;
	float maxSpeedR;
	float maxStandPitch;
	float maxLegLength;
	float stepDuration;
	float footYDistance;
	float stopPosThresholdX;
	float stopPosThresholdY;
	float stopSpeedThresholdX;
	float stopSpeedThresholdY;
	float zmpLeftX;
	float zmpLeftY;
	float zmpRightX;
	float zmpRightY;
	int outFilterOrder;
	float tiltFactor;
	float rollFactor;
	float tiltPFactor;
	float rollPFactor;
	int sensorDelay;
	int halSensorDelay;
	float maxFootSpeed;
	float fallDownAngle;
	float polygonLeft[4];
	float polygonRight[4];
	float offsetLeft[6];
	float offsetRight[6];
	int walkRequestFilterLen;
	float maxAccX;
	int accDelayX;
	float maxAccY;
	int accDelayY;
	float maxAccR;
	int accDelayR;
	int speedApplyDelay;
	//hardness settings
	int legJointHardness[6];
	float heightPolygon[5];
	// kick settings
	float normalizeFactor;
	float kickStart;
	float kickStop;
	float angleTiltP;
	float ballXMin;
	float ballXMax;
	float ballYMin;
	float ballYMax;
	float rotMin;
	float rotMax;
	float zmpMin;
	float zmpMax;
	float waitForGoodZMPFrames;
	float zmpMoveSpeedY;

	/** Constructor */
	WalkingEngineParams(){};

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(footPitch)
		STREAM(maxWalkPitch)
		STREAM(pitchSpeed)
		STREAM(xOffset)
		STREAM(stepHeight)
		STREAM(sensorControlRatio)
		STREAM(doubleSupportRatio)
		STREAM(crouchingDownPhaseLength)
		STREAM(startingPhaseLength)
		STREAM(stoppingPhaseLength)
		STREAM(armFactor)
		STREAM(arms1)
		STREAM(zmpSmoothPhase)
		STREAM(maxSpeedXForward)
		STREAM(maxSpeedXForwardOmni)
		STREAM(maxSpeedXBack)
		STREAM(maxSpeedYLeft)
		STREAM(maxSpeedYRight)
		STREAM(maxSpeedR)
		STREAM(maxStandPitch)
		STREAM(maxLegLength)
		STREAM(stepDuration)
		STREAM(footYDistance)
		STREAM(stopPosThresholdX)
		STREAM(stopPosThresholdY)
		STREAM(stopSpeedThresholdX)
		STREAM(stopSpeedThresholdY)
		STREAM(zmpLeftX)
		STREAM(zmpLeftY)
		STREAM(zmpRightX)
		STREAM(zmpRightY)
		STREAM(outFilterOrder)
		STREAM(tiltFactor)
		STREAM(rollFactor)
		STREAM(tiltPFactor)
		STREAM(rollPFactor)
		STREAM(sensorDelay)
		STREAM(halSensorDelay)
		STREAM(maxFootSpeed)
		STREAM(fallDownAngle)
		STREAM_ARRAY(polygonLeft)
		STREAM_ARRAY(polygonRight) 
		STREAM_ARRAY(offsetLeft)
		STREAM_ARRAY(offsetRight)
		STREAM(walkRequestFilterLen)
		STREAM(maxAccX)
		STREAM(accDelayX)
		STREAM(maxAccY)
		STREAM(accDelayY)
		STREAM(maxAccR)
		STREAM(accDelayR)
		STREAM(speedApplyDelay)
		STREAM_ARRAY(legJointHardness)
		STREAM_ARRAY(heightPolygon)
		STREAM(normalizeFactor)
		STREAM(kickStart)
		STREAM(kickStop)
		STREAM(angleTiltP)
		STREAM(ballXMin)
		STREAM(ballXMax)
		STREAM(ballYMin)
		STREAM(ballYMax)
		STREAM(rotMin)
		STREAM(rotMax)
		STREAM(zmpMin)
		STREAM(zmpMax)
		STREAM(waitForGoodZMPFrames)
		STREAM(zmpMoveSpeedY)
		STREAM_REGISTER_FINISH();
	};
	/** Descructor */
	~WalkingEngineParams()
	{};

};

class FreeLegPhaseParams : public WalkingEngineParams { };

#endif
