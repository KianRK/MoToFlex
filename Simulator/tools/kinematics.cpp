#include "math/Pose3D.h"
#include "math/Common.h"
#include "kinematics.h"

float neckOffsetZ = 0.112;
float shoulderOffsetY = 0.098;
float upperArmLength = 0.090;
float lowerArmLength = 0.135;
float shoulderOffsetZ = 0.100;
float hipOffsetZ = 0.085;
float hipOffsetY = 0.050;
float thighLength = 0.100;
float tibiaLength = 0.100;
float footHeight = 0.046;

void extractPose(const Pose3D& matrix, double& x, double& y, double& z, double& roll, double& pitch, double& yaw) {
    // Translation
    x = matrix.translation[0];
    y = matrix.translation[1];
    z = matrix.translation[2];

    // Rotation
    if (std::abs(matrix.rotation[2][0]) != 1) {
        pitch = -std::asin(matrix.rotation[2][0]);
        roll = std::atan2(matrix.rotation[2][1] / std::cos(pitch), matrix.rotation[2][2] / std::cos(pitch));
        yaw = std::atan2(matrix.rotation[1][0] / std::cos(pitch), matrix.rotation[0][0] / std::cos(pitch));
    } else {
        // Gimbal lock case
        yaw = 0;
        if (matrix.rotation[2][0] == -1) {
            pitch = M_PI / 2;
            roll = std::atan2(matrix.rotation[0][1], matrix.rotation[0][2]);
        } else {
            pitch = -M_PI / 2;
            roll = std::atan2(-matrix.rotation[0][1], -matrix.rotation[0][2]);
        }
    }
}

void forwardKinematics(bool leftLeg, double t0, double t1, double t2, double t3, double t4, double t5, double pose[])
{
	double sign = -1;
    if (leftLeg == 0)
		sign = 1;

	double tiltAngleOfFirstJointAxis =- pi / 2 + sign * pi / 4;
	double cx = cos(tiltAngleOfFirstJointAxis);
	double sx = sin(tiltAngleOfFirstJointAxis);

	Vector3<double> nullTrans(0, 0, 0);
	Vector3<double> xAxis(1, 0, 0);
	Vector3<double> yAxis(0, 1, 0);
	Vector3<double> zAxis(0, 0, 1);
	RotationMatrix noRot;

	
	Pose3D M1 = Pose3D(RotationMatrix(Matrix3x3<double>(
		1, 0, 0,
		0, cx, -sx,
		0, sx, cx
	)));

	Pose3D M2 = Pose3D(RotationMatrix(Matrix3x3<double>(
		1, 0, 0,
		0, cx, sx,
		0, -sx, cx
	)));

	Pose3D R0 = Pose3D(RotationMatrix(zAxis, t0), nullTrans);
	Pose3D R1 = Pose3D(RotationMatrix(xAxis, sign * t1), nullTrans);
	Pose3D R2 = Pose3D(RotationMatrix(yAxis, t2), nullTrans);
    Pose3D R3 = Pose3D(RotationMatrix(yAxis, t3), nullTrans); 
	Pose3D R4 = Pose3D(RotationMatrix(yAxis, t4), nullTrans);
	Pose3D R5 = Pose3D(RotationMatrix(xAxis, sign * t5), nullTrans); 

	Pose3D T0 = Pose3D(noRot, Vector3<double>(0, -sign * hipOffsetY, -hipOffsetZ));  
	Pose3D T1 = Pose3D(noRot, Vector3<double>(0, 0, -thighLength));
	Pose3D T2 = Pose3D(noRot, Vector3<double>(0, 0, -tibiaLength));
	Pose3D T3 = Pose3D(noRot, Vector3<double>(0, 0, -footHeight));

	Pose3D homMatrix = T0 * M1 * R0 * M2 * R1 * R2 * T1 * R3 * T2 * R4 * R5 * T3;

    extractPose(homMatrix, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}

void t0InverseKinematics(bool leftLeg, double x, double y, double z, double rx, double ry, double t0, float angles[])
{
	double right = (leftLeg ? -1 : 1);

	double xsign = 1;
	if (x < 0)
		xsign = -1;

	double tiltAngleOfFirstJointAxis =- pi / 2 + right * pi / 4;
 
 /*
    cx = np.cos(tiltAngleOfFirstJointAxis)
    sx = np.sin(tiltAngleOfFirstJointAxis)
    M1 = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]])
    M2 = np.array([[1, 0, 0],
                   [0, cx, sx],
                   [0, -sx, cx]])
*/

	double cx = cos(tiltAngleOfFirstJointAxis);
	double sx = sin(tiltAngleOfFirstJointAxis);

	Matrix3x3<double> M1(
		1, 0, 0,
		0, cx, -sx,
		0, sx, cx
	);

	Matrix3x3<double> M2(
		1, 0, 0,
		0, cx, sx,
		0, -sx, cx
	);

	// BEGIN	footRotX=rotMatrixAroundX(right*rx);
	//			footRot=rotMatrixAroundY(ry);
	//			footHeightVec=footRot*footRotX*[0;0;46];
	RotationMatrix footRot, footRotX;
	Vector3<double> footHeightVec(0, 0, footHeight);
	footRotX.rotateX(right*rx);
	footRot.rotateY(ry);
	footHeightVec = footRot * footRotX * footHeightVec;

	// END

	// BEGIN p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);
	RotationMatrix rotZ;
	rotZ.rotateZ(-t0);

	Vector3<double> p(x, y + right * hipOffsetY, z + hipOffsetZ);
	p = p + footHeightVec;
	p = M1 * rotZ * M2 * p;
	// END p = M1 * rotMatrixAroundZ(-t0)*M2*([x;y;z]+[0;0;46]+[0;right*50;85]);

	double ysign = -1*right;

	double r = p.abs();
	if (r>0.2)
	{
		//OUTPUT(idText, text, "Bein zu kurz!\r\n");
		p.normalize(0.2);
		r=0.2;
		//Vector3<double>(0, 0, -hipOffsetZ) + M1 * rotZ * M2 * Vector3<double>(

	}

	double t3 = 2 * asin((r / 2) / tibiaLength);
	double t1 = atan(p[1]/p[2]);

	Vector3<double> p1(p);
	p1.x = 0;
	double t2a = (pi-t3)/2;

	double soll;
	if (p[0]!=0)
		soll=atan(p[0]/p1.abs());
	else
		soll=0;

	double t2=t2a+soll;


	tiltAngleOfFirstJointAxis = - pi / 2 +  pi / 4;

	M1.c[1].y = cos(tiltAngleOfFirstJointAxis);
	M1.c[2].y = -sin(tiltAngleOfFirstJointAxis);
	M1.c[1].z = sin(tiltAngleOfFirstJointAxis);
	M1.c[2].z = cos(tiltAngleOfFirstJointAxis);

	M2.c[1].y = cos(tiltAngleOfFirstJointAxis);
	M2.c[2].y = sin(tiltAngleOfFirstJointAxis);
	M2.c[1].z = -sin(tiltAngleOfFirstJointAxis);
	M2.c[2].z = cos(tiltAngleOfFirstJointAxis);

	// BEGIN R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);
	Matrix3x3<double> R;
	RotationMatrix rotZ2, rotY4, rotX, rotY5;
	rotZ2.rotateZ(t0);
	rotY4.rotateY(-t2);
	rotX.rotateX(ysign * t1);
	rotY5.rotateY(pi - t3);
	R = M1 * rotZ2 * M2 * rotX * rotY4 * rotY5;
	// END R=M1*rotMatrixAroundZ(t0)*M2; R=R*rotMatrixAroundX(ysign*t1)*rotMatrixAroundY(-t2)*rotMatrixAroundY(pi-t3);

	// BEGIN schnitt=cross(R*[0;1;0], [0;0;1]);
	Vector3<double> v3(0, 0, 1);
	Vector3<double> v4(0, 1, 0);
	v4 = R * v4;
	Vector3<double> schnitt(v4 ^ (footRot * v3));
	// END schnitt=cross(R*[0;1;0], [0;0;1]);


	Vector3<double> vek(R * v3);

	double t4 = -asin((schnitt * vek) / (schnitt.abs() * vek.abs()));

	// BEGIN R=R*rotMatrixAroundY(t4);
	RotationMatrix rotY6;
	rotY6.rotateY(t4);
	R = R * rotY6;
	// END R=R*rotMatrixAroundY(t4);

	// BEGIN schnitt=cross(R*[1;0;0], [0;0;1]);
	Vector3<double> v5(1, 0, 0);
	schnitt = (R * v5) ^ (footRot * v3);
	// END schnitt=cross(R*[1;0;0], [0;0;1]);

	vek = R * v3;

	// BEGIN t5=-asin((schnitt'*vek)/(norm(schnitt)*norm(vek)))+rx;
	double t5 = -asin((schnitt * vek) / (schnitt.abs() * vek.abs()))+right*rx;
	// END

	angles[0] = t0; // [-90 ;  0 ] = [...
	angles[1] = ysign * t1; // [-100; 25 ] = [...
	angles[2] = t2; // [-45 ; 25 ]
	angles[3] = -(pi - t3); // [ 0  ; 130]
	angles[4] = -t4; // [-75 ; 45 ]
	angles[5] = t5; // [-25 ; 45 ]	
}

void inverseKinematics(bool leftLeg, double x, double y, double z, double rx, double ry, double rz, float angles[])
{
	double sign = (leftLeg ? -1 : 1);

	RotationMatrix rot;
	rot.rotateX(rx);
	rot.rotateY(ry);
	rot.rotateZ(rz);

	Vector3<double> position(x, y, z);
	Pose3D T(rot,position);

	//    T = T(Waist->J0) T0to5 T(J5->Foot)
	// => T0to5 = T(Waist->J0)^-1 T T(J5->Foot)^-1
	// with M1 = T(Waist->J0)^-1
	//      M2 = T(J5->Foot)^-1

	Pose3D M1(0, - sign * hipOffsetY, - hipOffsetZ);
	M1 = M1.invert();
	Pose3D M2(0,0, - footHeight);
	M2 = M2.invert();

	Pose3D T0to5 = M1 * T * M2;


	// now let's calculate the angles
	double t0,t1,t2,t3,t4,t5;
	double angle1,angle2; // will be needed later

	double length = T0to5.translation.abs(); // sqrt( T0to5(1,4) *  T0to5(1,4) + T0to5(2,4) * T0to5(2,4) + T0to5(3,4) * T0to5(3,4) );

	if (length > thighLength + tibiaLength)
	{
		t3 = 0;
		angle1 = 0;
		angle2 = 0;
	}
	else
	{
		// first the knee angle by law of cosines
		double kneeAngle = acos( (tibiaLength*tibiaLength + thighLength*thighLength - length*length) / (2*thighLength*tibiaLength) );
		t3 = pi - kneeAngle; // Will be a positive number. t3 might theoretically also be negative, but the knee joint can only be bent in one direction, so this is the correct one.

		angle1 = asin( thighLength * sin(kneeAngle) / length); // law of sines
		angle2 = pi - kneeAngle - angle1;
	}


	Pose3D T0to5Inv = T0to5.invert();
	t5 = atan2( T0to5Inv.translation.y, T0to5Inv.translation.z ); // atan2(T0to5Inv(2,4),T0to5Inv(3,4));
	t4 = atan2( -T0to5Inv.translation.x, sqrt(T0to5Inv.translation.y*T0to5Inv.translation.y + T0to5Inv.translation.z*T0to5Inv.translation.z) ) - angle1;


	//    T0to5 = T0to2 * T2to3 * T3to4 * T4to5
	// => T0to2 = T0to5 * T4to5^-1 * T3to4^-1 * T2to3^-1

	RotationMatrix dummy; // somehow the static call doesn't work; has to be investigated
	Pose3D T4to5Inv(dummy.fromRotationX(-t5));
	Pose3D T3to4Inv(dummy.fromRotationY(-t4));
	Pose3D T2to3Inv(dummy.fromRotationY(-t3));

	Pose3D T0to2 = ((T0to5 * T4to5Inv) * T3to4Inv) * T2to3Inv;


	double c0,s0,c1,s1,c2,s2;



	double _x = 1 / sqrt(2.0);

	double X1 = T0to2.rotation[0][0]; // = T0to2(1,1);
	//double X2 = T0to2(1,2);
	double X3 = T0to2.rotation[2][0]; // = T0to2(1,3);
	double X4 = T0to2.rotation[0][1]; // = T0to2(2,1);
	double X5 = T0to2.rotation[1][1]; // = T0to2(2,2);
	double X6 = T0to2.rotation[2][1]; // = T0to2(2,3);
	double X7 = T0to2.rotation[0][2]; // = T0to2(3,1);
	double X8 = T0to2.rotation[1][2]; // = T0to2(3,2);
	double X9 = T0to2.rotation[2][2]; // = T0to2(3,3);

	double X10 = sign * X6 + X9; // = c2(c1-s1)
	double X11 = sign * X4 + X7; // = s2(s1-c1)
	double X12 = X5 - sign * X8; // = c0(c1-s1)

	// X10 might be zero in the following cases:
	// 1.: c2 might be 0. This cannot be, because t2 is in [-100,25] and therefore can neither be -180 nor 180.
	// 2.: (c1-s1) might be 0. This can happen if t1 is 45 (or -135, but this is not possible).
	if (X10 == 0)
	{
		// ERROR HANDLING NEEDED!
		OUTPUT(idText,text,"something wrong in NaoKinematic");
    t0 = t1 = t2 = 0;
	}
	else // X10 != 0   => c2 != 0  && (c1-s1) != 0
	{
		t2 = atan( - X11 / X10 );
		s2 = sin(t2);
		c2 = cos(t2);

		c0 = c2 * X1 + s2 * X3;
		s0 = -(c2 * X7 + s2 * X9) / _x;
		t0 = atan2(s0,c0);

		if (c0 == 0)
		{
			c1 = X5 + X10/(2*c2); // c1 != 0, see above
			s1 = X5 - X10/(2*c2);
		}
		else if (s2 == 0) // && c0 != 0)
		{
			double X14 = X5 - sign * X6; // = s1 + c0c1
			s1 = (X14 - X12) / (1 + c0); // c0 won't be -1, because t0 can neither be -180 nor 180
			c1 = (X14 + X12/c0) / (1 + c0);
		}
		else // (c0 != 0) && (s0 != 0)
		{
			double X13 = - X7/s2 - sign* X6/c2 + _x*s0*s2/c2 - _x*s0*c2/s2; // = c0(c1+s1)
			c1 = (X12 + X13)/(2*c0); // c0 != 0, because "else"...
			s1 = (-X12 + X13)/(2*c0);
		}
		t1 = atan2(s1,c1);
	}

	// sign's are necessary because bredo-framework uses a different joint calibration than the rotation directions shown in the above mentioned figure provided by aldebaran.
	angles[0] = t0; // [-90 ;  0 ] = [...
	angles[1] = t1; // [-100; 25 ] = [...
	angles[2] = -t2; // [-45 ; 25 ]
	angles[3] = -t3; // [ 0  ; 130]
	angles[4] = -t4; // [-75 ; 45 ]
	angles[5] = sign * t5; // [-25 ; 45 ]
}
