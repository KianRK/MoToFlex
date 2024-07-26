/**
 * @file Pose2D.cpp
 * Implementation of class Pose2D
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#include "Pose2D.h"

#ifdef Pose2Dnew

In& operator>>(In& stream, RotationVector& rotationVector)
{
  stream >> rotationVector.sinus;
  stream >> rotationVector.cosinus;
  return stream;
}

Out& operator<<(Out& stream, const RotationVector& rotationVector)
{
  stream << rotationVector.sinus;
  stream << rotationVector.cosinus;
  return stream;
}

#endif //Pose2DNew

In& operator>>(In& stream, Pose2D& pose2D)
{
  STREAM_REGISTER_BEGIN_EXT(pose2D);
  STREAM_EXT(stream, pose2D.rotation);
  STREAM_EXT(stream, pose2D.translation);
  STREAM_REGISTER_FINISH();
  return stream;
}

Out& operator<<(Out& stream, const Pose2D& pose2D)
{
  STREAM_REGISTER_BEGIN_EXT(pose2D);
  STREAM_EXT(stream, pose2D.rotation);
  STREAM_EXT(stream, pose2D.translation);
  STREAM_REGISTER_FINISH();
  return stream;
}
