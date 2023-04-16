'''
Created on 2019. 3. 15.

@author: JSK
'''
import numpy as np
from math import *
import scipy


def rad2deg(rads):
    return rads/np.pi*180
        
def deg2rad(degs):
    return degs/180*np.pi

def Rot_axis( axis, q ):
    '''
    make rotation matrix along axis
    '''
    if axis==1:
        R = np.asarray([[1,0,0],[0,cos(q),-sin(q)],[0,sin(q),cos(q)]])
    if axis==2:
        R = np.asarray([[cos(q),0,sin(q)],[0,1,0],[-sin(q),0,cos(q)]])
    if axis==3:
        R = np.asarray([[cos(q),-sin(q),0],[sin(q),cos(q),0],[0,0,1]])
    return R

def Rot_zyx(zr, yr, xr):
    '''
    zyx rotatio matrix - caution: axis order: z,y,x
    '''
    R = np.matmul(np.matmul(Rot_axis(3, zr), Rot_axis(2, yr)), Rot_axis(1, xr))
    return R

def Rot_zxz(zr1,xr2,zr3):
    '''
    zxz rotatio matrix - caution: axis order: z,x,z
    '''
    R = np.matmul(np.matmul(Rot_axis(3,zr1),Rot_axis(1,xr2)),Rot_axis(3,zr3))
    return R

def Rot2zyx(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,0]**2 + R[1,0]**2)

    if sy > 0.000001:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.asarray([z,y,x])

def Rot2zxz(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,2]**2 + R[1,2]**2)

    if sy > 0.000001:
        z1 = atan2(R[0,2] , -R[1,2])
        x2 = atan2(sy,R[2,2])
        z3 = atan2(R[2,0], R[2,1])
    else:
        z1 = 0
        x2 = atan2(sy,R[2,2])
        z3 = atan2(-R[0,1], R[0,0])
    return np.asarray([z1,x2,z3])

def SE3(R, P):
    T = np.identity(4, dtype='float32')
    T[0:3, 0:3] = R
    T[0:3, 3] = P
    return T
    
def SE3_inv(T):
    R = T[0:3,0:3].transpose()
    P=-np.matmul(R,T[0:3,3])
    return (SE3(R,P))
    
def SE3_R(T):
    return T[0:3, 0:3]
    
def SE3_P(T):
    return T[0:3, 3]
   
def SE3_mul_vec3(T,v):
    r=np.matmul(SE3_R(T),v)
    return np.add(r,SE3_P(T))

def average_SE3(Ts):
    nT = Ts.shape[0]
    Rref = Ts[0,:3,:3]
    dRlie_list = []
    for i in range(nT):
        Ri = Ts[i,:3,:3]
        dRi = np.matmul(Rref.transpose(),Ri)
        dRlie = np.real(scipy.linalg.logm(dRi,disp=False)[0])
        dRlie_list += [dRlie]
    dRlie_list = np.array(dRlie_list)
    dRlie_m = np.mean(dRlie_list,axis=0)
    R_m = np.matmul(Rref,scipy.linalg.expm(dRlie_m))
    P_m = np.mean(Ts[:,:3,3],axis=0)
    T_m=SE3(R_m,P_m)
    return T_m

def align_z(Two):
    Rwo = Two[0:3,0:3]
    Zwo=np.matmul(Rwo,[[0],[0],[1]])
    azim=np.arctan2(Zwo[1],Zwo[0])-np.deg2rad(90)
    altit=np.arctan2(np.linalg.norm(Zwo[:2]),Zwo[2])
    Rwo_=np.matmul(Rot_zxz(azim,altit,-azim),Rwo)
    Two_out = Two.copy()
    Two_out[0:3,0:3] = Rwo_
    return Two_out

def fit_floor(Tcw, Tco, minz):
    Pco = Tco[0:3,3]
    Twc = np.linalg.inv(Tcw)
    Pco_wz = np.dot(Twc[2,0:3],Pco)
    if abs(Pco_wz)<0.00001:
        Pco_wz = 0.00001
    alpha = abs((-minz - Twc[2,3])/Pco_wz)
    Pco_ = Pco*alpha
    Tco_out = Tco.copy()
    Tco_out[0:3,3]=Pco_
    return Tco_out

def cmd2T(pos_):
    R_ = Rot_zyx(radians(pos_[5]), radians(pos_[4]), radians(pos_[3]))
    P_ = np.array([pos_[0], pos_[1], pos_[2]])
    T_ = SE3(R_, P_)
    return T_

def T2cmd(T_):
    angle_ = Rot2zyx(SE3_R(T_))
    pos_ = np.zeros((6,))
    pos_[0] = SE3_P(T_)[0]
    pos_[1] = SE3_P(T_)[1]
    pos_[2] = SE3_P(T_)[2]
    pos_[3] = degrees(angle_[2])
    pos_[4] = degrees(angle_[1])
    pos_[5] = degrees(angle_[0])
    return pos_

def rot_x180(_T):
    Tx180 = np.identity(4, 'float32')
    Tx180[1, 1] = -1
    Tx180[2, 2] = -1
    return np.matmul(_T, Tx180)

def rot_x90(_T):
    Tx90 = np.zeros((4, 4), 'float32')
    Tx90[1, 2] = -1
    Tx90[2, 1] = 1
    Tx90[0, 0] = 1
    Tx90[3, 3] = 1
    return np.matmul(_T, Tx90)

def rot_xm90(_T):
    Tx90 = np.zeros((4, 4), 'float32')
    Tx90[1, 2] = 1
    Tx90[2, 1] = -1
    Tx90[0, 0] = 1
    Tx90[3, 3] = 1
    return np.matmul(_T, Tx90)

def rot_y180(_T):
    Ty180 = np.identity(4, 'float32')
    Ty180[0, 0] = -1
    Ty180[2, 2] = -1
    return np.matmul(_T, Ty180)

def rot_y90(_T):
    Ty90 = np.zeros((4, 4), 'float32')
    Ty90[0, 2] = -1
    Ty90[2, 0] = 1
    Ty90[1, 1] = 1
    Ty90[3, 3] = 1
    return np.matmul(_T, Ty90)

def rot_ym90(_T):
    Ty90 = np.zeros((4, 4), 'float32')
    Ty90[0, 2] = 1
    Ty90[2, 0] = -1
    Ty90[1, 1] = 1
    Ty90[3, 3] = 1
    return np.matmul(_T, Ty90)

def rot_z180(_T):
    Tz180 = np.identity(4, 'float32')
    Tz180[0, 0] = -1
    Tz180[1, 1] = -1
    return np.matmul(_T, Tz180)

def rot_z90(_T):
    Tz90 = np.zeros((4, 4), 'float32')
    Tz90[0, 1] = -1
    Tz90[1, 0] = 1
    Tz90[2, 2] = 1
    Tz90[3, 3] = 1
    return np.matmul(_T, Tz90)

def rot_zm90(_T):
    Tz90 = np.zeros((4, 4), 'float32')
    Tz90[0, 1] = 1
    Tz90[1, 0] = -1
    Tz90[2, 2] = 1
    Tz90[3, 3] = 1
    return np.matmul(_T, Tz90)


Tx180 = np.identity(4, 'float32')
Tx180[1, 1] = -1
Tx180[2, 2] = -1

Ty180 = np.identity(4, 'float32')
Ty180[0, 0] = -1
Ty180[2, 2] = -1



