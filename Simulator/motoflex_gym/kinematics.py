from scipy.spatial.transform import Rotation
import numpy as np
import math
class naoConfiguration:
    NeckOffsetZ = 0.112
    ShoulderOffsetY = 0.098
    UpperArmLength = 0.090
    LowerArmLength = 0.135
    ShoulderOffsetZ = 0.100
    HipOffsetZ = 0.085
    HipOffsetY = 0.050
    ThighLength = 0.100
    TibiaLength = 0.100
    FootHeight = 0.046

def rotMatrixAroundZ(angle):
    c = np.cos(angle)
    s = np.sin(angle)

    M = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])

    return M

def rotMatrixAroundY(angle):
    c = np.cos(angle)
    s = np.sin(angle)

    M = np.array([[c, 0, s],
                  [0, 1, 0],
                  [-s, 0, c]])

    return M

def rotMatrixAroundX(angle):
    c = np.cos(angle)
    s = np.sin(angle)

    M = np.array([[1, 0, 0],
                  [0, c, -s],
                  [0, s, c]])

    return M

def translationMatrix(x, y, z):
    M = np.array([[1, 0, 0, x],
                  [0, 1, 0, y],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]])

    return M

def from3x3toHomCoords(M):
    Mh = np.array([[M[0,0], M[0,1], M[0,2], 0],
                   [M[1,0], M[1,1], M[1,2], 0],
                   [M[2,0], M[2,1], M[2,2], 0],
                   [0, 0, 0, 1]])
    
    return Mh

def extract_angles_and_positions(matrix):
    r = Rotation.from_matrix(matrix[:3, :3])
    angles = r.as_euler('xyz', degrees=False)
    position = matrix[:3, 3]
    return *position, *angles

def normalize(vector, length):
    if np.linalg.norm(vector) == 0:
        return vector
    return (vector * length) / np.linalg.norm(vector)

def getTransformationFromCoordinates(x, y, z, rx, ry, rz):
    translation = np.array([x, y, z])
    rotation_x = rotMatrixAroundX(rx)
    rotation_y = rotMatrixAroundY(ry)
    rotation_z = rotMatrixAroundZ(rz)
    
    transformation = np.eye(4)
    transformation[:3, 3] = translation
    transformation[:3, :3] = rotation_z @ rotation_y @ rotation_x
    
    return transformation

def specialIKNewJointConf(x, y, z, rx, ry, t0, right):
    xsign = 1
    if x < 0:
        xsign = -1

    tiltAngleOfFirstJointAxis = -np.pi / 2 + right * np.pi / 4

    cx = np.cos(tiltAngleOfFirstJointAxis)
    sx = np.sin(tiltAngleOfFirstJointAxis)
    M1 = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]])
    M2 = np.array([[1, 0, 0],
                   [0, cx, sx],
                   [0, -sx, cx]])

    footRotX = rotMatrixAroundX(rx)
    footRot = rotMatrixAroundY(ry)
    footHeightVec = footRot @ footRotX @ np.array([0, 0, naoConfiguration.FootHeight])
    p = M1 @ rotMatrixAroundZ(-t0) @ M2 @ (np.array([x, y, z]) + 
                                           np.array([0, 
                                                     right * naoConfiguration.HipOffsetY, 
                                                     naoConfiguration.HipOffsetZ]) + 
                                           footHeightVec)

    ysign = -1 * right
    l = naoConfiguration.ThighLength # Not sure, maybe the TibiaLength?
    r = np.linalg.norm(p)
    
    if r > 0.2:
        p = normalize(p, 0.2)
        r = 0.2

    t1 = np.arctan(p[1] / p[2])
    t3 = 2 * np.arcsin((r / 2) / l)
    t2a = (np.pi - t3) / 2

    p1 = p.copy()
    p1[0] = 0

    if p[0] != 0:
        soll = np.arctan(p[0] / np.linalg.norm(p1))
    else:
        soll = 0

    t2 = t2a + soll

    tiltAngleOfFirstJointAxis = -np.pi / 2 + np.pi / 4
    cx = np.cos(tiltAngleOfFirstJointAxis)
    sx = np.sin(tiltAngleOfFirstJointAxis)
    M1 = np.array([[1, 0, 0],
                   [0, cx, -sx],
                   [0, sx, cx]])
    M2 = np.array([[1, 0, 0],
                   [0, cx, sx],
                   [0, -sx, cx]])
    R = M1 @ rotMatrixAroundZ(t0) @ M2
    R = R @ rotMatrixAroundX(ysign * t1) @ rotMatrixAroundY(-t2) @ rotMatrixAroundY(np.pi - t3)

    schnitt = np.cross(R @ np.array([0, 1, 0]), footRot @ np.array([0, 0, 1]))
    vek = R @ np.array([0, 0, 1])
    t4 = -np.arcsin((schnitt @ vek) / (np.linalg.norm(schnitt) * np.linalg.norm(vek)))

    R = R @ rotMatrixAroundY(t4)

    schnitt = np.cross(R @ np.array([1, 0, 0]), footRot @ np.array([0, 0, 1]))
    vek = R @ np.array([0, 0, 1])
    t5 = -np.arcsin((schnitt @ vek) / (np.linalg.norm(schnitt) * np.linalg.norm(vek))) + right * rx

    A = [
        float(t0),
        ysign * float(t1),
        float(t2),
        - np.pi + float(t3),
        - float(t4),
        float(t5)
    ]

    return A

def inverseKinematicNewJointConf(x, y, z, rx, ry, rz, right):
    t0 = 0
    t1 = 0
    t2 = 0
    t3 = 0
    t4 = 0
    t5 = 0

    sign = 1
    if right == 0:
        sign = -1

    T = getTransformationFromCoordinates(x, y, z, rx, ry, rz)

    M1 = np.linalg.inv(translationMatrix(0, -sign * naoConfiguration.HipOffsetY, -naoConfiguration.HipOffsetZ))
    M2 = np.linalg.inv(translationMatrix(0, 0, -naoConfiguration.FootHeight))

    T0to5 = M1 @ T @ M2

    length = np.sqrt(T0to5[0, 3] ** 2 + T0to5[1, 3] ** 2 + T0to5[2, 3] ** 2)

    if length > (naoConfiguration.ThighLength + naoConfiguration.TibiaLength):
        t3 = 0
        angle1 = 0
        angle2 = 0
    else:
        kneeAngle = np.arccos((naoConfiguration.TibiaLength ** 2 + naoConfiguration.ThighLength ** 2 - length ** 2) / (2 * naoConfiguration.ThighLength * naoConfiguration.TibiaLength))
        t3 = np.pi - kneeAngle
        angle1 = np.arcsin(naoConfiguration.ThighLength * np.sin(kneeAngle) / length)
        angle2 = np.pi - kneeAngle - angle1

    T0to5Inv = np.linalg.inv(T0to5)
    t5 = np.arctan2(T0to5Inv[1, 3], T0to5Inv[2, 3])
    t4 = np.arctan2(-T0to5Inv[0, 3], np.sqrt(T0to5Inv[2, 3] ** 2 + T0to5Inv[1, 3] ** 2)) - angle1

    T4to5Inv = np.linalg.inv(from3x3toHomCoords(rotMatrixAroundX(t5)))
    T3to4Inv = np.linalg.inv(from3x3toHomCoords(rotMatrixAroundY(t4)))
    T2to3Inv = np.linalg.inv(from3x3toHomCoords(rotMatrixAroundY(t3)))

    T0to2 = ((T0to5 @ T4to5Inv) @ T3to4Inv) @ T2to3Inv

    x = 1 / np.sqrt(2.0)

    X1 = T0to2[0, 0]
    X2 = T0to2[0, 1]
    X3 = T0to2[0, 2]
    X4 = T0to2[1, 0]
    X5 = T0to2[1, 1]
    X6 = T0to2[1, 2]
    X7 = T0to2[2, 0]
    X8 = T0to2[2, 1]
    X9 = T0to2[2, 2]

    X10 = sign * X6 + X9
    X11 = sign * X4 + X7
    X12 = X5 - sign * X8

    t2 = np.arctan(-X11 / X10)
    s2 = np.sin(t2)
    c2 = np.cos(t2)

    c0 = c2 * X1 + s2 * X3
    s0 = -(c2 * X7 + s2 * X9) / x
    t0 = np.arctan2(s0, c0)

    if c0 != 0 and s2 != 0:
        X13 = -X7 / s2 - sign * X6 / c2 + x * s0 * s2 / c2 - x * s0 * c2 / s2
        c1 = (X12 + X13) / (2 * c0)
        s1 = (-X12 + X13) / (2 * c0)
        t1 = np.arctan2(s1, c1)
    else:
        t1 = 0

    A = [
        float(t0),
        float(t1),
        float(-t2),
        float(-t3),
        float(-t4),
        float(sign * t5)
    ]

    return A

def spherical_to_cart(theta, phi, r):
    x = r * math.sin(theta) * math.cos(phi)
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(theta)
    return np.array([x, y, z])

def cart_to_spherical(x, y, z):
    r = math.sqrt(x**2 + y**2 + z**2)
    theta = math.acos(z / r)  # inclination
    phi = math.atan2(y, x)    # azimuth
    return np.array([theta, phi, r])

def trans_mat_to_spherical(matrix):
    x, y, z = matrix[:3, 3]
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / r)  # inclination
    phi = np.arctan2(y, x)    # azimuth
    return np.array([theta, phi, r])

def forwardsKinematicsNewJointConf(t0, t1, t2, t3, t4, t5, right):
    sign = 1
    if right == 0:
        sign = -1

    tiltAngleOfFirstJointAxis = -np.pi / 2 + sign * np.pi / 4
    cx = np.cos(tiltAngleOfFirstJointAxis)
    sx = np.sin(tiltAngleOfFirstJointAxis)
    M1 = np.array([[1, 0, 0, 0],
                   [0, cx, -sx, 0],
                   [0, sx, cx, 0],
                   [0, 0, 0, 1]])
    M2 = np.array([[1, 0, 0, 0],
                   [0, cx, sx, 0],
                   [0, -sx, cx, 0],
                   [0, 0, 0, 1]])
    R0 = from3x3toHomCoords(rotMatrixAroundZ(t0))
    R1 = from3x3toHomCoords(rotMatrixAroundX(sign * t1))
    R2 = from3x3toHomCoords(rotMatrixAroundY(t2))
    R3 = from3x3toHomCoords(rotMatrixAroundY(t3))
    R4 = from3x3toHomCoords(rotMatrixAroundY(t4))
    R5 = from3x3toHomCoords(rotMatrixAroundX(sign * t5))

    T0 = translationMatrix(0, -sign * naoConfiguration.HipOffsetY, -naoConfiguration.HipOffsetZ)
    T1 = translationMatrix(0, 0, -naoConfiguration.ThighLength)
    T2 = translationMatrix(0, 0, -naoConfiguration.TibiaLength)
    T3 = translationMatrix(0, 0, -naoConfiguration.FootHeight)

    hom_matrix = T0 @ M1 @ R0 @ M2 @ R1 @ R2 @ T1 @ R3 @ T2 @ R4 @ R5 @ T3

    return hom_matrix

#leftLegAngles = inverseKinematicNewJointConf(0, 50, -230, 0, 0, 0, 0)
#print(leftLegAngles)
#leftLeg6D = forwardsKinematicsNewJointConf(*leftLegAngles, 0)
#print(leftLeg6D)
#rightLegAngles = specialIKNewJointConf(0, -50, -230, 0, 0, leftLegAngles[0], 1)
#print(rightLegAngles)
#rightLeg6D = forwardsKinematicsNewJointConf(*leftLegAngles, 1)
#print(rightLeg6D)
