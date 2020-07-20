from IPython.core.debugger import Tracer; keyboard = Tracer()
import numpy as np


def A(x,y,z):
    return np.array([[1, 0, 0, x],[0, 1, 0, y],[0, 0, 1, z],[0, 0, 0, 1]])

def T_DH(a, alpha, d, theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, a], \
                     [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)], \
                     [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.sin(alpha), -np.cos(alpha), d*np.cos(alpha)], \
                     [0, 0, 0, 1]])
def R_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0, 0], \
                     [np.sin(theta), np.cos(theta), 0, 0], \
                     [0, 0, 1, 0], \
                     [0, 0, 0, 1]])

def transform(RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, Rotation):
    #transform x,y,z in base axis to end effectors' x, y, z
    #expects angle is radian
    ShoulderOffsetY = 98.00
    ElbowOffsetY = 15.00
    ShoulderOffsetZ = 100.0
    UpperArmLength = 105.00
    HandOffsetX = 57.75
    LowerArmLength = 55.95
    X = np.array([[0, 0, 0, 1]]).T
    transMat = A(0,-ShoulderOffsetY-ElbowOffsetY, ShoulderOffsetZ)
    transMat = np.dot(transMat, T_DH(0, -np.pi/2.0, 0, RShoulderPitch))
    transMat = np.dot(transMat, T_DH(0, np.pi/2.0, 0, RShoulderRoll+np.pi/2.0))
    transMat = np.dot(transMat, T_DH(0, -np.pi/2.0, -UpperArmLength, RElbowYaw))
    transMat = np.dot(transMat, T_DH(0, np.pi/2.0, 0, RElbowRoll))
    transMat = np.dot(transMat, R_z(np.pi/2.0))
    transMat = np.dot(transMat, A(-HandOffsetX-LowerArmLength, 0, 0))
    transMat = np.dot(transMat, R_z(-np.pi))
    resultMat = np.dot(transMat, X)

    return [ resultMat[0], resultMat[1], resultMat[2] ]

"""
angles is a ndarray(T, angle_num). Angle value should be radian (de-normalized)
"""
def simDraw(angles):
    line = np.zeros((angles.shape[0],2))
    
    for t, ang in enumerate(angles):
        pos = transform(ang[0], ang[1], ang[2], ang[3], ang[4])
        line[t,0] = pos[1] * -1.0 # X
        line[t,1] = pos[0] * -1.0 # Y
        
    return line
