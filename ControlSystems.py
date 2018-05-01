# -*- coding: utf-8 -*-
"""
Spyder Editor

Created by: Joseph (Tripp) Cannella
"""

class PID():
    prevValue = 0
    integralValue = 0
    proportionalGain = 0
    integralGain = 0
    derivativeGain = 0
    totalGain = 0
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def updateGain(self, currentValue, deltaT):
        self.proportionalGain = currentValue*self.Kp
        self.integralValue += currentValue
        self.integralGain = self.integralValue*self.Ki*deltaT
        self.derivativeGain = self.Kd*(currentValue - self.prevValue)/deltaT
        self.prevValue = currentValue
        self.totalGain = self.proportionalGain + self.integralGain + self.derivativeGain
        return self.totalGain