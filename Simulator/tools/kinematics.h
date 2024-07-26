#pragma once
void forwardKinematics(bool leftLeg, double t0, double t1, double t2, double t3, double t4, double t5, double pose[]);
void t0InverseKinematics(bool leftLeg, double x, double y, double z, double rx, double ry, double t0, float angles[]);
void inverseKinematics(bool leftLeg, double x, double y, double z, double rx, double ry, double rz, float angles[]);

