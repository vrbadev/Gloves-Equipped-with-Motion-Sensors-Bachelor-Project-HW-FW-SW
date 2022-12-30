# -*- coding: utf-8 -*-
"""
9-DOF Kalman filter implementation
(Missing implementation for 6-DOF sensors)

Created on Thu May  7 16:14:29 2020
@author: Vojtech Vrba, FEE CTU in Prague, vrbavoj3@fel.cvut.cz
"""

import numpy as np

class Kalman():
    STATE_SIZE = 4
    OUTPUT_SIZE = 6
    Q_VARIANCE = 0.01
    R_VARIANCE_ACC = 25.0
    R_VARIANCE_MAG = 50.0
    
    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        
        self.gravX = 0.0
        self.gravY = 0.0
        self.gravZ = 0.0
        
        # The covariance matrix
        self.P = np.eye(Kalman.STATE_SIZE) * Kalman.Q_VARIANCE
        self.R = np.array([Kalman.R_VARIANCE_ACC, Kalman.R_VARIANCE_ACC, Kalman.R_VARIANCE_ACC, Kalman.R_VARIANCE_MAG, Kalman.R_VARIANCE_MAG, Kalman.R_VARIANCE_MAG])
        
        self.isCalibrated = False
    
    
    def update(self, acc, gyr, mag, dt):
        gx, gy, gz = gyr
        ax, ay, az = acc
        mx, my, mz = mag
         
        if not (mx == 0.0 and my == 0.0 and mz == 0.0):
            self.UpdateQImpl(gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz,  dt)
        else:
            self.update_imu(acc, gyr, dt)
        
        self.estimatedGravityDirection()
        if not self.isCalibrated:
            self.baseZacc = self.GetAccZ(ax, ay, az)
            self.isCalibrated = True
    
     
    def UpdateQImpl(self, gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz,  dt):
        gx = gx * np.pi / 180.0
        gy = gy * np.pi / 180.0
        gz = gz * np.pi / 180.0
         
        # Normalise accelerometer measurement
        recipNorm = (ax*ax + ay*ay + az*az) ** (-1/2)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm
         
        # Normalise magnetometer measurement
        recipNorm = (mx*mx + my*my + mz*mz) ** (-1/2)
        mx *= recipNorm
        my *= recipNorm
        mz *= recipNorm
         
        #Converto il vettore intensità campo magnetico
        # h1 =  mx*self.q1 + my*self.q2 + mz*self.q3 
        # h2 =  mx*self.q0 - my*self.q3 + mz*self.q2 
        # h3 =  mx*self.q3 + my*self.q0 - mz*self.q1 
        # h4 = -mx*self.q2 + my*self.q1 + mz*self.q0 
        # n2 = self.q0*h2 + self.q1*h1 + self.q2*h4 - self.q3*h3
        # n3 = self.q0*h3 - self.q1*h4 + self.q2*h1 + self.q3*h2
        # n4 = self.q0*h4 + self.q1*h3 - self.q2*h2 + self.q3*h1
        # b2 = arm_sqrt(n2*n2 + n3*n3)
        # b4 = n4
        # b2 = mx
        # b4 = mz
        hx = mx*self.q0*self.q0 + 2.0*mz*self.q0*self.q2 - 2.0*my*self.q0*self.q3 + mx*self.q1*self.q1 + 2.0*my*self.q1*self.q2 + 2.0*mz*self.q1*self.q3 - mx*self.q2*self.q2 - mx*self.q3*self.q3
        hy = my*self.q0*self.q0 - 2.0*mz*self.q0*self.q1 + 2.0*mx*self.q0*self.q3 - my*self.q1*self.q1 + 2.0*mx*self.q1*self.q2 + my*self.q2*self.q2 + 2.0*mz*self.q2*self.q3 - my*self.q3*self.q3
        hz = mz*self.q0*self.q0 + 2.0*my*self.q0*self.q1 - 2.0*mx*self.q0*self.q2 - mz*self.q1*self.q1 + 2.0*mx*self.q1*self.q3 - mz*self.q2*self.q1 + 2.0*my*self.q2*self.q3 + mz*self.q3*self.q3
        b2 = np.sqrt(hx*hx + hy*hy)
        b4 = hz
        
        A = np.zeros((Kalman.STATE_SIZE, Kalman.STATE_SIZE))
        A[0][0] = 1.0
        A[0][1] = -gx*dt*0.5
        A[0][2] = -gy*dt*0.5
        A[0][3] = -gz*dt*0.5
        A[1][0] = gx*dt*0.5
        A[1][1] = 1.0
        A[1][2] = gz*dt*0.5
        A[1][3] = -gy*dt*0.5
        A[2][0] = gy*dt*0.5
        A[2][1] = -gz*dt*0.5
        A[2][2] = 1.0
        A[2][3] = gx*dt*0.5
        A[3][0] = gz*dt*0.5
        A[3][1] = gy*dt*0.5
        A[3][2] = -gx*dt*0.5
        A[3][3] = 1.0
         
        # Aggiorno la Covarianza
        tmpNN1m = A.dot(self.P)
        tmpNN2m = np.transpose(A)
        P0 = tmpNN1m.dot(tmpNN2m) + np.eye(Kalman.STATE_SIZE) * Kalman.Q_VARIANCE
         
        # Calcolo il guadagno
        K = np.zeros((Kalman.STATE_SIZE, Kalman.OUTPUT_SIZE))
        h = np.zeros((Kalman.OUTPUT_SIZE, Kalman.STATE_SIZE))
         
        #Accelerometer
        h[0][0] = -2.0*self.q2
        h[0][1] =  2.0*self.q3
        h[0][2] = -2.0*self.q0
        h[0][3] =  2.0*self.q1
        h[1][0] =  2.0*self.q1
        h[1][1] =  2.0*self.q0
        h[1][2] =  2.0*self.q3
        h[1][3] =  2.0*self.q2
        h[2][0] =  4.0*self.q0
        h[2][1] =  0.0
        h[2][2] =  0.0
        h[2][3] =  4.0*self.q3
        #h[2][0] =  2.0*self.q0
        #h[2][1] = -2.0*self.q1
        #h[2][2] = -2.0*self.q2
        #h[2][3] =  2.0*self.q3
         
        #Magnetometer
        h[3][0] =  4.0*b2*self.q0 - 2.0*b4*self.q2
        h[3][1] =  4.0*b2*self.q1 + 2.0*b4*self.q3
        h[3][2] = -2.0*b4*self.q0
        h[3][3] =  2.0*b4*self.q1
        h[4][0] =  2.0*b4*self.q1 - 2.0*b2*self.q3
        h[4][1] =  2.0*b2*self.q2 + 2.0*b4*self.q0
        h[4][2] =  2.0*b2*self.q1 + 2.0*b4*self.q3
        h[4][3] =  2.0*b4*self.q2 - 2.0*b2*self.q0
        h[5][0] =  2.0*b2*self.q2 + 4.0*b4*self.q0
        h[5][1] =  2.0*b2*self.q3
        h[5][2] =  2.0*b2*self.q0
        h[5][3] =  2.0*b2*self.q1 + 4.0*b4*self.q3
         
        #h[3][0] = -2.0*b4*self.q2
        #h[3][1] =  2.0*b4*self.q3
        #h[3][2] = -4.0*b2*self.q2 - 2.0*b4*self.q0
        #h[3][3] = -4.0*b2*self.q3 + 2.0*b4*self.q1
        #h[4][0] = -2.0*b2*self.q3 + 2.0*b4*self.q1
        #h[4][1] =  2.0*b2*self.q2 + 2.0*b4*self.q0
        #h[4][2] =  2.0*b2*self.q1 + 2.0*b4*self.q3
        #h[4][3] = -2.0*b2*self.q0 + 2.0*b4*self.q2
        #h[5][0] =  2.0*b2*self.q2
        #h[5][1] =  2.0*b2*self.q3 - 4.0*b4*self.q1
        #h[5][2] =  2.0*b2*self.q0 - 4.0*b4*self.q2
        #h[5][3] =  2.0*b2*self.q1
         
         
        # ====== INNOVATION COVARIANCE ======
        HT = np.transpose(h)
        P0HT = P0.dot(HT)
        HP0HT = h.dot(P0HT)
        
        for i in range(Kalman.OUTPUT_SIZE): # Add the element of HPH' to the above
          HP0HT[i][i] += self.R[i]
        
        HP0HT_INVm = np.linalg.inv(HP0HT)  # (H*P0*H' + R)^(-1)
        K = P0HT.dot(HP0HT_INVm) # K = P0*H'*(H*P0*H' + R)^(-1)
         
        #Aggiorno Stato
        State = np.zeros((Kalman.STATE_SIZE, 1))
        Error = np.zeros((Kalman.OUTPUT_SIZE, 1))
        
        State[0][0] = self.q0
        State[1][0] = self.q1
        State[2][0] = self.q2
        State[3][0] = self.q3
        StateTMP = A.dot(State) # A*x(k)
         
        Error[0][0] = ax - 2.0*(self.q1*self.q3 - self.q0*self.q2)
        Error[1][0] = ay - 2.0*(self.q0*self.q1 + self.q2*self.q3)
        Error[2][0] = az - 2.0*(self.q0*self.q0 + self.q3*self.q3 - 0.5)
        Error[3][0] = mx - 2.0*(b2*(self.q0*self.q0 + self.q1*self.q1 - 0.5) - b4*(self.q0*self.q2 - self.q1*self.q3))
        Error[4][0] = my - 2.0*(b4*(self.q0*self.q1 + self.q2*self.q3) - b2*(self.q0*self.q3 - self.q1*self.q2))
        Error[5][0] = mz - 2.0*(b4*(self.q0*self.q0 + self.q3*self.q3 - 0.5) + b2*(self.q0*self.q2 + self.q1*self.q3))
        ErrorTMP = K.dot(Error)
         
        self.q0 = StateTMP[0][0] + ErrorTMP[0][0]
        self.q1 = StateTMP[1][0] + ErrorTMP[1][0]
        self.q2 = StateTMP[2][0] + ErrorTMP[2][0]
        self.q3 = StateTMP[3][0] + ErrorTMP[3][0]
         
        #Aggiorno la P
        tmpNN1 = K.dot(h)
        for i in range(Kalman.STATE_SIZE):
          tmpNN1[i][i] -= 1.0 
          for j in range(Kalman.STATE_SIZE):
              tmpNN1[i][j] *= -1.0 
        self.P = tmpNN1.dot(P0)
         
        # Normalise quaternion
        recipNorm = (self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3) ** (-1/2)
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm
    
    def update_imu(self, acc, gyr, dt):
        gx, gy, gz = gyr
        ax, ay, az = acc
        
        # [MISSING IMPLEMENTATION FOR 6-DOF]
        
    def computeAngles(self):
        gx, gy, gz = self.gravX, self.gravY, self.gravZ
        
        if gx >  1: 
            gx =  1
        if gx < -1: 
            gx = -1
        
        yaw = np.arctan2(2.0*(self.q0*self.q3 + self.q1*self.q2), self.q0*self.q0 + self.q1*self.q1 - self.q2*self.q2 - self.q3*self.q3) 
        pitch = np.arcsin(gx)  #Pitch seems to be inverted
        roll = np.arctan2(gy, gz)
        
        return roll, pitch, yaw
      
    
    def GetQuaternion(self):
      return self.q0, self.q1, self.q2, self.q3
     
    def GetAccZWithoutGravity(self, ax, ay, az):
      return self.GetAccZ(ax, ay, az) - self.baseZacc
     
    def GetInvThrustCompensationForTilt(self):
      # Return the z component of the estimated gravity direction
      # (0, 0, 1) dot G
      return self.gravZ
    
     
    def GetAccZ(self, ax, ay, az):
      # return vertical acceleration
      # (A dot G) / |G|,  (|G| = 1) -> (A dot G)
      return (ax * self.gravX + ay * self.gravY + az * self.gravZ)
    
    def estimatedGravityDirection(self):
      self.gravX = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
      self.gravY = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
      self.gravZ = self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3
  