#ifndef _ODOMETRYCORRECTION_H
#define _ODOMETRYCORRECTION_H
#include "Tools/Streams/Streamable.h"

class OdometryCorrectionTable : public Streamable
{
private :
	int entries;
	struct OdometryInfo : public Streamable
	{
		OdometryInfo() : speed(400),multiplier(1.0){}

		void serialize(In* in,Out* out)
		{
			STREAM_REGISTER_BEGIN();
			STREAM(speed)
			STREAM(multiplier)
			STREAM_REGISTER_FINISH();
		};
		double speed;
		double multiplier;
	};
	OdometryInfo odometryTable[10];

public:


	OdometryCorrectionTable()
	{
		entries = 10;
	}

	~OdometryCorrectionTable()
	{
	}
	//corrects the motions
	double correct(double value,double executedSpeed)
	{
		if(value<0.0001) return value;
		int sign = executedSpeed>0?1:-1;
		double absExecutedSpeed=(executedSpeed*sign);	
		double	currentMultiplier=1.;
		if (absExecutedSpeed >= odometryTable[entries-1].speed)
				currentMultiplier=odometryTable[entries-1].multiplier;
		else if (absExecutedSpeed <= odometryTable[0].speed)
				currentMultiplier=odometryTable[0].multiplier;
		else{
			int index =1;
			while (index<(entries-1)&&absExecutedSpeed>=(odometryTable[index].speed)) index++;
			//calculate regression line 
			double	regStartMultiplier=odometryTable[index-1].multiplier;
			double	regStartSpeed=odometryTable[index-1].speed;
			double	regEndSpeed=odometryTable[index].speed;
			double	regEndMultiplier=odometryTable[index].multiplier;
			double	multiplierDiff=(regEndMultiplier-regStartMultiplier);
			double	speedDiff=(regEndSpeed-regStartSpeed);
			if(multiplierDiff!=0&&speedDiff!=0){
				currentMultiplier=(multiplierDiff/speedDiff)*(absExecutedSpeed-regStartSpeed)+regStartMultiplier;
			}else{
				currentMultiplier=regStartMultiplier;
			}
		}
		return currentMultiplier * value;
	}

	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(entries);
		STREAM_ARRAY(odometryTable);
		STREAM_REGISTER_FINISH();
	}

};
#endif
