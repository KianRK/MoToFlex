/**
* @file SensorCalibration.h
* Declaration of a class for representing the calibration values of sensors.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas R�fer</a>
*/

#ifndef __SensorCalibration_H__
#define __SensorCalibration_H__

#include "Tools/Streams/Streamable.h"

class SensorCalibration : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(accXOffset);
    STREAM(accXGain);
    STREAM(accYOffset);
    STREAM(accYGain);
    STREAM(accZOffset);
    STREAM(accZGain);
    STREAM(batteryLow);
    STREAM(temperatureHigh);
    STREAM(gyroXOffset);
    STREAM(gyroXGain);
    STREAM(gyroYOffset);
    STREAM(gyroYGain);
    STREAM(gyroZOffset);
    STREAM(gyroZGain);
    STREAM(compassXOffset);
    STREAM(compassYOffset);
    STREAM(fsrLFLOffset);
    STREAM(fsrLFLGain);
    STREAM(fsrLFROffset);
    STREAM(fsrLFRGain);
    STREAM(fsrLBLOffset);
    STREAM(fsrLBLGain);
    STREAM(fsrLBROffset);
    STREAM(fsrLBRGain);
    STREAM(fsrRFLOffset);
    STREAM(fsrRFLGain);
    STREAM(fsrRFROffset);
    STREAM(fsrRFRGain);
    STREAM(fsrRBLOffset);
    STREAM(fsrRBLGain);
    STREAM(fsrRBROffset);
    STREAM(fsrRBRGain);
    STREAM_REGISTER_FINISH()
  }

public:
  float accXOffset; /**< The correction offset in g. */
  float accXGain; /**< The factor between sensor units and g. */
  float accYOffset; /**< The correction offset in g. */
  float accYGain; /**< The factor between sensor units and g. */
  float accZOffset; /**< The correction offset in g. */
  float accZGain; /**< The factor between sensor units and g. */
  float batteryLow; /**< The voltage below which the robot is switched off. */
  int temperatureHigh; /**< The temperature the robot starts complaining about the temperature. */
  float gyroXOffset; /**< The correction offset in g. */
  float gyroXGain; /**< The factor between sensor units and g. */
  float gyroYOffset; /**< The correction offset in g. */
  float gyroYGain; /**< The factor between sensor units and g. */
  float gyroZOffset; /**< The correction offset in g. */
  float gyroZGain; /**< The factor between sensor units and g. */
  float compassXOffset; 
  float compassYOffset; 
  float fsrLFLOffset;
  float fsrLFLGain;
  float fsrLFROffset;
  float fsrLFRGain;
  float fsrLBLOffset;
  float fsrLBLGain;
  float fsrLBROffset;
  float fsrLBRGain;
  float fsrRFLOffset;
  float fsrRFLGain;
  float fsrRFROffset;
  float fsrRFRGain;
  float fsrRBLOffset;
  float fsrRBLGain;
  float fsrRBROffset;
  float fsrRBRGain;

  /** 
  * Default constructor.
  */
  SensorCalibration() :
    accXOffset(0),
    accXGain(1),
    accYOffset(0),
    accYGain(1),
    accZOffset(0),
    accZGain(1),
    batteryLow(100),
    gyroXGain(1),
    gyroYGain(1),
    gyroZGain(1),
    compassXOffset(0),
    compassYOffset(0),
    fsrLFLOffset(0),
    fsrLFLGain(1),
    fsrLFROffset(0),
    fsrLFRGain(1),
    fsrLBLOffset(0),
    fsrLBLGain(1),
    fsrLBROffset(0),
    fsrLBRGain(1),
    fsrRFLOffset(0),
    fsrRFLGain(1),
    fsrRFROffset(0),
    fsrRFRGain(1),
    fsrRBLOffset(0),
    fsrRBLGain(1),
    fsrRBROffset(0),
    fsrRBRGain(1) {}
};

#endif
