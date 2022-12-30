# -*- coding: utf-8 -*-
"""
Madgwick's implementation of Mahony's AHRS algorithm.
See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Created on Thu May  7 00:09:59 2020
@author: Vojtech Vrba, FEE CTU in Prague, vrbavoj3@fel.cvut.cz
"""

import numpy as np


class Mahony():
    twoKp = (2.0 * 0.5)	# 2 * proportional gain
    twoKi =	(2.0 * 0.0)	# 2 * integral gain

    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.integralFBx = 0.0
        self.integralFBy = 0.0
        self.integralFBz = 0.0


    def update(self, acc, gyr, mag, delta_t):
        gx, gy, gz = gyr
        ax, ay, az = acc
        mx, my, mz = mag
	
        # Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533
    
        # Use IMU algorithm if magnetometer measurement invalid
        # (avoids NaN in magnetometer normalisation)
        if mx == 0.0 and my == 0.0 and mz == 0.0:
        	self.updateIMU(acc, gyr, delta_t)
        	return
        
        
        # Compute feedback only if accelerometer measurement valid
        # (avoids NaN in accelerometer normalisation)
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        	# Normalise accelerometer measurement
        	recipNorm = (ax * ax + ay * ay + az * az) ** (-1/2)
        	ax *= recipNorm
        	ay *= recipNorm
        	az *= recipNorm
            
        	# Normalise magnetometer measurement
        	recipNorm = (mx * mx + my * my + mz * mz) ** (-1/2)
        	mx *= recipNorm
        	my *= recipNorm
        	mz *= recipNorm
            
        	# Auxiliary variables to avoid repeated arithmetic
        	q0q0 = self.q0 * self.q0
        	q0q1 = self.q0 * self.q1
        	q0q2 = self.q0 * self.q2
        	q0q3 = self.q0 * self.q3
        	q1q1 = self.q1 * self.q1
        	q1q2 = self.q1 * self.q2
        	q1q3 = self.q1 * self.q3
        	q2q2 = self.q2 * self.q2
        	q2q3 = self.q2 * self.q3
        	q3q3 = self.q3 * self.q3
            
        	# Reference direction of Earth's magnetic field
        	hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
        	hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
        	bx = np.sqrt(hx * hx + hy * hy)
        	bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))
            
        	# Estimated direction of gravity and magnetic field
        	halfvx = q1q3 - q0q2
        	halfvy = q0q1 + q2q3
        	halfvz = q0q0 - 0.5 + q3q3
        	halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
        	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
        	halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)
            
        	# Error is sum of cross product between estimated direction
        	# and measured direction of field vectors
        	halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
        	halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
        	halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)
            
        	# Compute and apply integral feedback if enabled
        	if Mahony.twoKi > 0.0:
        		# integral error scaled by Ki
        		self.integralFBx += Mahony.twoKi * halfex * delta_t
        		self.integralFBy += Mahony.twoKi * halfey * delta_t
        		self.integralFBz += Mahony.twoKi * halfez * delta_t
        		gx += self.integralFBx	# apply integral feedback
        		gy += self.integralFBy
        		gz += self.integralFBz
        	else:
        		self.integralFBx = 0.0	# prevent integral windup
        		self.integralFBy = 0.0
        		self.integralFBz = 0.0
            
        	# Apply proportional feedback
        	gx += Mahony.twoKp * halfex
        	gy += Mahony.twoKp * halfey
        	gz += Mahony.twoKp * halfez
            
        # Integrate rate of change of quaternion
        gx *= (0.5 * delta_t)		# pre-multiply common factors
        gy *= (0.5 * delta_t)
        gz *= (0.5 * delta_t)
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
        self.q1 += (qa * gx + qc * gz - self.q3 * gy)
        self.q2 += (qa * gy - qb * gz + self.q3 * gx)
        self.q3 += (qa * gz + qb * gy - qc * gx)
            
        # Normalise quaternion
        recipNorm = (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3) ** (-1/2)
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm

    #-------------------------------------------------------------------------------------------
    # IMU algorithm update

    def updateIMU(self, acc, gyr, delta_t):
        gx, gy, gz = gyr
        ax, ay, az = acc
        
        # Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533

        # Compute feedback only if accelerometer measurement valid
        # (avoids NaN in accelerometer normalisation)
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        	# Normalise accelerometer measurement
        	recipNorm = (ax * ax + ay * ay + az * az) ** (-1/2)
        	ax *= recipNorm
        	ay *= recipNorm
        	az *= recipNorm
            
        	# Estimated direction of gravity
        	halfvx = self.q1 * self.q3 - self.q0 * self.q2
        	halfvy = self.q0 * self.q1 + self.q2 * self.q3
        	halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3
            
        	# Error is sum of cross product between estimated
        	# and measured direction of gravity
        	halfex = (ay * halfvz - az * halfvy)
        	halfey = (az * halfvx - ax * halfvz)
        	halfez = (ax * halfvy - ay * halfvx)
            
        	# Compute and apply integral feedback if enabled
        	if Mahony.twoKi > 0.0:
        		# integral error scaled by Ki
        		self.integralFBx += Mahony.twoKi * halfex * delta_t
        		self.integralFBy += Mahony.twoKi * halfey * delta_t
        		self.integralFBz += Mahony.twoKi * halfez * delta_t
        		gx += self.integralFBx	# apply integral feedback
        		gy += self.integralFBy
        		gz += self.integralFBz
        	else:
        		self.integralFBx = 0.0	# prevent integral windup
        		self.integralFBy = 0.0
        		self.integralFBz = 0.0
            
        	# Apply proportional feedback
        	gx += Mahony.twoKp * halfex
        	gy += Mahony.twoKp * halfey
        	gz += Mahony.twoKp * halfez
            
        # Integrate rate of change of quaternion
        gx *= (0.5 * delta_t)		# pre-multiply common factors
        gy *= (0.5 * delta_t)
        gz *= (0.5 * delta_t)
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
        self.q1 += (qa * gx + qc * gz - self.q3 * gy)
        self.q2 += (qa * gy - qb * gz + self.q3 * gx)
        self.q3 += (qa * gz + qb * gy - qc * gx)
            
        # Normalise quaternion
        recipNorm = (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3) ** (-1/2)
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm


    #-------------------------------------------------------------------------------------------
    
    def computeAngles(self):
        roll = np.arctan2(self.q0*self.q1 + self.q2*self.q3, 0.5 - self.q1*self.q1 - self.q2*self.q2)
        pitch = np.arcsin(-2.0 * (self.q1*self.q3 - self.q0*self.q2))
        yaw = np.arctan2(self.q1*self.q2 + self.q0*self.q3, 0.5 - self.q2*self.q2 - self.q3*self.q3)
        return roll, pitch, yaw
    