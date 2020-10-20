#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Peng Longxiang <jason@ufactory.cc> <penglongxiang1988@126.com>

import sys
import numpy as np
import math

def get_sign(x):
	if x>=0:
		return 1;
	return -1

# Transfer from given rpy_angles:[roll, pitch, yaw] to rotation matrix Rot
def rpy_to_rot(rpy_angles):
	Rx = np.mat([[1, 0, 0],[ 0, math.cos(rpy_angles[0]), -math.sin(rpy_angles[0])],[ 0, math.sin(rpy_angles[0]), math.cos(rpy_angles[0])]])
	Ry = np.mat([[math.cos(rpy_angles[1]),0, math.sin(rpy_angles[1])],[ 0, 1, 0],[ -math.sin(rpy_angles[1]), 0, math.cos(rpy_angles[1])]])
	Rz = np.mat([[math.cos(rpy_angles[2]), -math.sin(rpy_angles[2]), 0],[ math.sin(rpy_angles[2]), math.cos(rpy_angles[2]), 0],[ 0, 0, 1]])

	Rot = Rz*Ry*Rx
	return Rot

# Transfer from given rotation matrix RotM to [roll, pitch, yaw] angles
def rot_to_rpy(RotM):
	pitch = math.atan2(-RotM[2,0], math.sqrt(RotM[2,1]**2 + RotM[2,2]**2))
	sign = get_sign(math.cos(pitch))
	roll = math.atan2(RotM[2,1]*sign, RotM[2,2]*sign)
	yaw = math.atan2(RotM[1,0]*sign, RotM[0,0]*sign)
	return [roll, pitch, yaw]

# Transfer given Angle-Axis [Rx, Ry, Rz] representation to rotation matrix Rot
def angleAxis_to_rot(Rx, Ry, Rz):
	phi = math.sqrt(Rx**2+Ry**2+Rz**2) # this is the rotation angle along the rotation axis
	if not abs(phi)>1e-5:
		return np.mat(np.zeros((3,3)))

	Axis = [Rx/phi, Ry/phi, Rz/phi] # this is the unit vector of rotation
	# skew-symmetric Matrix 
	ss = np.mat([[0, -Axis[2], Axis[1]], [Axis[2], 0, -Axis[0]], [-Axis[1], Axis[0], 0]])
	Rot = np.mat(np.eye(3)) + math.sin(phi)*ss + (1-math.cos(phi))*ss**2
	return Rot

# Transfer given rotation matrix Rot to angle-Axis representation
def rot_to_angleAxis(RotM):
	if (RotM == np.mat(np.eye(3))).all():
		phi = 0
		Axis = [0, 0, 0]
	elif (RotM[0,0] + RotM[1,1] + RotM[2,2]) == -1:
		phi = math.pi
		denom = math.sqrt(2*(1+RotM[2,2]))
		Axis = [RotM[0,2]/denom, RotM[1,2]/denom, (1+RotM[2,2])/denom]
	else:
		phi = math.acos(0.5*((RotM[0,0] + RotM[1,1] + RotM[2,2])-1))
		denom = 2*math.sin(phi)
		Axis = [ (RotM[2,1]-RotM[1,2])/denom,  (RotM[0,2]-RotM[2,0])/denom, (RotM[1,0]-RotM[0,1])/denom]

	return [phi*Axis[0], phi*Axis[1], phi*Axis[2]]


def angleAxis_to_rpy(Rx, Ry, Rz):
	M = angleAxis_to_rot(Rx, Ry, Rz)
	rpy = rot_to_rpy(M)
	return rpy

def rpy_to_angleAxis(rpy):
	Rot = rpy_to_rot(rpy)
	AA = rot_to_angleAxis(Rot)
	return AA


if __name__ == '__main__':

	# NOTE: all angle units are in RADIANS
	# Test: input the angle-Axis (UR) orientation command, and transfer to roll-pitch-yaw angles (xArm)
	# Angle-axis is not a unit vector because it is multiplied by the rotation angle along that unit vector

	rpy = angleAxis_to_rpy(1.2, -1.21, 1.2)
	print("Transferred Roll-Pitch-Yaw is: [%f, %f, %f]"%(rpy[0], rpy[1], rpy[2]))

	# For feedback, you can also transfer roll-pitch-yaw to angle-axis representation 
	AA = rpy_to_angleAxis(rpy)
	print("Transferred Angle-Axis is: [%f, %f, %f]"%(AA[0], AA[1], AA[2]))
