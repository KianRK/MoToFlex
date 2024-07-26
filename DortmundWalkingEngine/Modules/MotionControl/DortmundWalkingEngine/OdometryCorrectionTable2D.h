#ifndef _ODOMETRYCORRECTION2D_H
#define _ODOMETRYCORRECTION2D_H
#include "Tools/Streams/Streamable.h"
#include <vector>

//OdometryCorrectionTable2D currently only used for rotation
//Use for 2DOdometryCorrection not excluded but should be checked
 
class OdometryCorrectionTable2D : public Streamable
{
private :
	int entries;
	struct OdometryInfo2D : public Streamable
	{
		OdometryInfo2D() : speedX(400),speedR(200),multiplier(1.0){}

		void serialize(In* in,Out* out)
		{
			STREAM_REGISTER_BEGIN();
			STREAM(speedX)
			STREAM(speedR)
			STREAM(multiplier)
			STREAM_REGISTER_FINISH();
		};
		double speedX;
		double speedR;
		double multiplier;
	};
	
	struct OdometryInfo1D
	{
		OdometryInfo1D() : speed(400),multiplier(1.0){}

		double speed;
		double multiplier;
	};
	
	OdometryInfo2D odometryTable2D[10];	
	std::vector< std::vector< OdometryInfo1D > > odometrySorted;
	
	double correct1D(double executedSpeed,std::vector<OdometryInfo1D> odometryTable)
	{
		int sign = executedSpeed>0?1:-1;
		double absExecutedSpeed=(executedSpeed*sign);	
		double	currentMultiplier=1.;
		if (absExecutedSpeed >= odometryTable.at(odometryTable.size()-1).speed)
			currentMultiplier=odometryTable.at(odometryTable.size()-1).multiplier;
		if (absExecutedSpeed <= odometryTable.at(1).speed) 
			currentMultiplier=odometryTable.at(1).multiplier;
		else{
			unsigned int index =1;
			while (index<(odometryTable.size()-1)&&absExecutedSpeed>=(odometryTable.at(index).speed)) index++;
      if(index<2) return odometryTable.at(index).multiplier;

			//calculate regression line 
			double	regStartMultiplier=odometryTable.at(index-1).multiplier;
			double	regStartSpeed=odometryTable.at(index-1).speed;
			double	regEndSpeed=odometryTable.at(index).speed;
			double	regEndMultiplier=odometryTable.at(index).multiplier;
			double	multiplierDiff=(regEndMultiplier-regStartMultiplier);
			double	speedDiff=(regEndSpeed-regStartSpeed);
			if(multiplierDiff!=0&&speedDiff!=0){
				currentMultiplier=(multiplierDiff/speedDiff)*(absExecutedSpeed-regStartSpeed)+regStartMultiplier;
			}else{
				currentMultiplier=regStartMultiplier;
			}
		}
		return currentMultiplier;
	}

public:

	void sortData(){
		odometrySorted.clear();
		std::vector<OdometryInfo1D> oneSpeedXVector;
		
		//create 1DOdometry correction table for every speedX		
		double currentSpeedX=odometryTable2D[0].speedX;
		OdometryInfo1D temp;
			temp.speed=currentSpeedX;
			temp.multiplier=1.;
			oneSpeedXVector.push_back(temp);
		for (int i=0;i<entries;i++){
			if(currentSpeedX==odometryTable2D[i].speedX){
				
			temp.speed= odometryTable2D[i].speedR;
			temp.multiplier=odometryTable2D[i].multiplier;
			oneSpeedXVector.push_back(temp);
			}else{
				odometrySorted.push_back(oneSpeedXVector);
				oneSpeedXVector.clear();
				currentSpeedX=odometryTable2D[i].speedX;
				temp.speed=currentSpeedX;
				temp.multiplier=1.;
				oneSpeedXVector.push_back(temp);
				temp.speed= odometryTable2D[i].speedR;
				temp.multiplier=odometryTable2D[i].multiplier;
				oneSpeedXVector.push_back(temp);
			}
		}
		odometrySorted.push_back(oneSpeedXVector);
	}

	OdometryCorrectionTable2D()
	{
		entries = 10;
	}

	~OdometryCorrectionTable2D()
	{
	}
	//corrects the motions
	double correct(double value,double executedSpeedX, double executedSpeedR)
	{
		double currentMultiplier=1.;
		try
		{
			if(odometrySorted.empty())sortData();
			if(value<0.0001||odometrySorted.empty()||odometrySorted.at(0).empty()) return value;
			//create 1DOdometry correction table for every speedX			
			double absExecutedSpeedX=fabs(executedSpeedX);
			if(absExecutedSpeedX<=odometrySorted.at(0).at(0).speed)
				currentMultiplier=correct1D(executedSpeedR,odometrySorted.at(0));
			else if(absExecutedSpeedX>=odometrySorted.at(odometrySorted.size()-1).at(0).speed)
				currentMultiplier=correct1D(executedSpeedR,odometrySorted.at(odometrySorted.size()-1));
			else{
				unsigned int index =1;
				while (index<(odometrySorted.size()-1)&&absExecutedSpeedX>=(odometrySorted.at(index).at(0).speed)) index++;
				//calculate regression line 
				double regStartMultiplier=correct1D(executedSpeedR,odometrySorted.at(index-1));
				double regStartSpeed=odometrySorted.at(index-1).at(0).speed;
				double regEndMultiplier=correct1D(executedSpeedR,odometrySorted.at(index));
				double regEndSpeed=odometrySorted.at(index).at(0).speed;
				double multiplierDiff=(regEndMultiplier-regStartMultiplier);
				double speedDiff=(regEndSpeed-regStartSpeed);
				if(multiplierDiff!=0&&speedDiff!=0){
					currentMultiplier=(multiplierDiff/speedDiff)*(absExecutedSpeedX-regStartSpeed)+regStartMultiplier;
				}else{
					currentMultiplier=regStartMultiplier;
				}
			}
		}
    catch (std::string& ex)
		{
			OUTPUT(idText,text,"module:MotionCombinator:OdometryCorrection2D - "+ex);
		}
		return currentMultiplier * value;
	}
	
	void serialize(In* in,Out* out)
	{
		STREAM_REGISTER_BEGIN();
		STREAM(entries);
		STREAM_ARRAY(odometryTable2D);
		STREAM_REGISTER_FINISH();
	}

};
#endif
