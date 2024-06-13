#!/usr/bin/python3

# https://github.com/mc-capolei/python-Universal-robot-kinematics/blob/master/universal_robot_kinematics.py
# https://github.com/MobMonRob/python-Universal-robot-Power-measurements/blob/master/Power_Calculator.py
# https://github.com/mhorstAK/python-Universal-robot-kinematics/tree/master
## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University

# ***** lib
import numpy as np
from numpy import linalg
import cmath
from math import cos, sin, atan2, acos, asin, sqrt, pi   

global mat
mat=np.matrix

robot_name = 'ur5' #ur5, ur10

# ****** Coefficients ******
global d, a, alpha

if robot_name == 'ur5':
  # d     = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
  # a     = mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0])
  # alpha = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ])
  dh = [
      # a,        alpha,        d,     theta
      [0.0,       0.5*pi,     0.089159,  pi],
      [-0.42500,  0.0,        0.0,      0.0],
      [-0.39225,  0.0,        0.0,      0.0],
      [0.0,       0.5*pi,     0.10915,  0.0],
      [0.0,       -0.5*pi,    0.09465,  0.0],
      [0.0,       0.0,        0.0823,   0.0],
  ]
elif robot_name == 'ur5e':
  dh = [
      # a,        alpha,        d,     theta
      [0.0,       0.5*pi,     0.163,     pi],
      [-0.425,    0.0,        0.0,      0.0],
      [-0.39225,  0.0,        0.0,      0.0],
      [0.0,       0.5*pi,     0.134,    0.0],
      [0.0,       -0.5*pi,    0.100,    0.0],
      [0.0,       0.0,        0.100,    0.0],
  ]
elif robot_name == 'ur10':
  # d     = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])  
  # a     = mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0]) 
  # alpha = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ])
  dh = [
      # a,        alpha,        d,     theta
      [0.0,       0.5*pi,     0.1273,     pi],
      [-0.612,    0.0,        0.0,       0.0],
      [-0.5723,   0.0,        0.0,       0.0],
      [0.0,       0.5*pi,     0.163941,  0.0],
      [0.0,       -0.5*pi,    0.1157,    0.0],
      [0.0,       0.0,        0.0922,    0.0],
  ]
else:
	print("No robot_name called ", robot_name)
       
# List type of D-H parameter
# Do not remove these
a     = np.array([dh[0][0], dh[1][0], dh[2][0], dh[3][0], dh[4][0], dh[5][0]]) # unit: mm
d     = np.array([dh[0][2], dh[1][2], dh[2][2], dh[3][2], dh[4][2], dh[5][2]]) # unit: mm
alpha = np.array([dh[0][1], dh[1][1], dh[2][1], dh[3][1], dh[4][1], dh[5][1]]) # unit: radian

# ************************************************** FORWARD KINEMATICS

def AH(n, th, c):

  # https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
  
  # Translation
  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[n-1]

  # Rotation
  theta_n = th[n-1,c]
  Rzt = mat([[cos(theta_n), -sin(theta_n), 0, 0],
  	         [sin(theta_n),  cos(theta_n), 0, 0],
  	         [0,             0,            1, 0],
  	         [0,             0,            0, 1]],copy=False)
      
  alpha_n = alpha[n-1]
  Rxa = mat([[1, 0,             0,              0],
	      		 [0, cos(alpha_n), -sin(alpha_n),   0],
	      		 [0, sin(alpha_n),  cos(alpha_n),   0],
	      		 [0, 0,             0,              1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
  # A_i = Rxa * T_a * Rzt * T_d

  return A_i

def forKine(th, c):  
  theta_n = pi
  Rz180 = mat([[cos(theta_n), -sin(theta_n), 0, 0],
               [sin(theta_n),  cos(theta_n), 0, 0],
               [0,             0,            1, 0],
               [0,             0,            0, 1]],copy=False)

  # UR5 /base
  # A_1=AH( 1, th, c)

  # UR5 /base_link
  A_1=Rz180*AH( 1, th, c)
  A_2=AH( 2, th, c)
  A_3=AH( 3, th, c)
  A_4=AH( 4, th, c)
  A_5=AH( 5, th, c)
  A_6=AH( 6, th, c)
      
  T_01=A_1
  T_02=A_1*A_2
  T_03=A_1*A_2*A_3
  T_04=A_1*A_2*A_3*A_4
  T_05=A_1*A_2*A_3*A_4*A_5
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_01,T_02,T_03,T_04,T_05,T_06

# ************************************************** INVERSE KINEMATICS 

def invKine(desired_pos):# T60
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0, 0, -d[5], 1]).T-mat([0, 0, 0, 1]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d[3] /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_16 = T_10 * desired_pos
    th[4, c:c+2]   = + acos((T_16[2,3]-d[3])/d[5])
    th[4, c+2:c+4] = - acos((T_16[2,3]-d[3])/d[5])

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_16 = linalg.inv( T_10 * desired_pos )
    th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
		  
  th = th.real

  # **** theta3 ****

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_65 = AH( 6,th,c)
    T_54 = AH( 5,th,c)
    T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
    P_13 = T_14 * mat([0, -d[3], 0, 1]).T - mat([0,0,0,1]).T
    t3 = cmath.acos((linalg.norm(P_13)**2 - a[1]**2 - a[2]**2 )/(2 * a[1] * a[2])) # norm ?
    th[2, c]   = t3.real
    th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH( 1,th,c ))
    T_65 = linalg.inv(AH( 6,th,c))
    T_54 = linalg.inv(AH( 5,th,c))
    T_14 = (T_10 * desired_pos) * T_65 * T_54
    P_13 = T_14 * mat([0, -d[3], 0, 1]).T - mat([0,0,0,1]).T
    
    # theta 2
    th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a[2]* sin(th[2,c])/linalg.norm(P_13))
    # theta 4
    T_32 = linalg.inv(AH( 3,th,c))
    T_21 = linalg.inv(AH( 2,th,c))
    T_34 = T_32 * T_21 * T_14
    th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real

  return th
