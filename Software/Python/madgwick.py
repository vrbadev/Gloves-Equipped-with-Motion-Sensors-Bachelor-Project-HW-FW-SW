# -*- coding: utf-8 -*-
"""
Implementation of Madgwick's IMU and AHRS algorithms.
See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Created on Wed May  6 23:38:52 2020
@author: Vojtech Vrba, FEE CTU in Prague, vrbavoj3@fel.cvut.cz
"""

import numpy as np

class Madgwick():
    beta       = 0.1    # 2 * proportional gain

    def __init__(self):
	    self.q0 = 1.0
	    self.q1 = 0.0
	    self.q2 = 0.0
	    self.q3 = 0.0
    

    def update(self, acc, gyr, mag, delta_t):
        gx, gy, gz = gyr
        ax, ay, az = acc
        mx, my, mz = mag
        
        # Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if mx == 0.0 and my == 0.0 and mz == 0.0:
        	self.updateIMU(gyr, acc, delta_t)
        	return
            
        # Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533
            
        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)
            
        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
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
        	_2q0mx = 2.0 * self.q0 * mx
        	_2q0my = 2.0 * self.q0 * my
        	_2q0mz = 2.0 * self.q0 * mz
        	_2q1mx = 2.0 * self.q1 * mx
        	_2q0 = 2.0 * self.q0
        	_2q1 = 2.0 * self.q1
        	_2q2 = 2.0 * self.q2
        	_2q3 = 2.0 * self.q3
        	_2q0q2 = 2.0 * self.q0 * self.q2
        	_2q2q3 = 2.0 * self.q2 * self.q3
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
        	hx = mx * q0q0 - _2q0my * self.q3 + _2q0mz * self.q2 + mx * q1q1 + _2q1 * my * self.q2 + _2q1 * mz * self.q3 - mx * q2q2 - mx * q3q3
        	hy = _2q0mx * self.q3 + my * q0q0 - _2q0mz * self.q1 + _2q1mx * self.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * self.q3 - my * q3q3
        	_2bx = np.sqrt(hx * hx + hy * hy)
        	_2bz = -_2q0mx * self.q2 + _2q0my * self.q1 + mz * q0q0 + _2q1mx * self.q3 - mz * q1q1 + _2q2 * my * self.q3 - mz * q2q2 + mz * q3q3
        	_4bx = 2.0 * _2bx
        	_4bz = 2.0 * _2bz
            
        	# Gradient decent algorithm corrective step
        	s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * self.q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q3 + _2bz * self.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        	s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * self.q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q2 + _2bz * self.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q3 - _4bz * self.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        	s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * self.q2 - _2bz * self.q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q1 + _2bz * self.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q0 - _4bz * self.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        	s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * self.q3 + _2bz * self.q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q0 + _2bz * self.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        	recipNorm = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) ** (-1/2) # normalise step magnitude
        	s0 *= recipNorm
        	s1 *= recipNorm
        	s2 *= recipNorm
        	s3 *= recipNorm
            
        	# Apply feedback step
        	qDot1 -= Madgwick.beta * s0
        	qDot2 -= Madgwick.beta * s1
        	qDot3 -= Madgwick.beta * s2
        	qDot4 -= Madgwick.beta * s3
            
        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += qDot1 * delta_t
        self.q1 += qDot2 * delta_t
        self.q2 += qDot3 * delta_t
        self.q3 += qDot4 * delta_t
            
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
        
        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)
            
        # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        	# Normalise accelerometer measurement
        	recipNorm = (ax * ax + ay * ay + az * az) ** (-1/2)
        	ax *= recipNorm
        	ay *= recipNorm
        	az *= recipNorm
            
        	# Auxiliary variables to avoid repeated arithmetic
        	_2q0 = 2.0 * self.q0
        	_2q1 = 2.0 * self.q1
        	_2q2 = 2.0 * self.q2
        	_2q3 = 2.0 * self.q3
        	_4q0 = 4.0 * self.q0
        	_4q1 = 4.0 * self.q1
        	_4q2 = 4.0 * self.q2
        	_8q1 = 8.0 * self.q1
        	_8q2 = 8.0 * self.q2
        	q0q0 = self.q0 * self.q0
        	q1q1 = self.q1 * self.q1
        	q2q2 = self.q2 * self.q2
        	q3q3 = self.q3 * self.q3
            
        	# Gradient decent algorithm corrective step
        	s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        	s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        	s2 = 4.0 * q0q0 * self.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        	s3 = 4.0 * q1q1 * self.q3 - _2q1 * ax + 4.0 * q2q2 * self.q3 - _2q2 * ay
        	recipNorm = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) ** (-1/2) # normalise step magnitude
        	s0 *= recipNorm
        	s1 *= recipNorm
        	s2 *= recipNorm
        	s3 *= recipNorm
            
        	# Apply feedback step
        	qDot1 -= Madgwick.beta * s0
        	qDot2 -= Madgwick.beta * s1
        	qDot3 -= Madgwick.beta * s2
        	qDot4 -= Madgwick.beta * s3
            
        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += qDot1 * delta_t
        self.q1 += qDot2 * delta_t
        self.q2 += qDot3 * delta_t
        self.q3 += qDot4 * delta_t
                
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
