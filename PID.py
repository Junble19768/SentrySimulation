import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from util import *

class PID:
    def __init__(self, 
                 p:float=0, i:float=0, d:float=0, 
                 nowValue:float=0, tarValue:float=0,
                 stackSize:int=10):
        self.p = p
        self.i = i
        self.d = d
        self.dSum = 0
        self.iSum = 0
        self.stackSize = stackSize
        self.detErrorStack = [0]*self.stackSize
        self.errorStack = [0]*self.stackSize
        self.nowIdx = 0
        self.preError = 0
        
        self.nowValue = nowValue
        self.tarValue = tarValue
        
    def measure(self, value:float):
        self.nowValue = value
        nowError = self.tarValue - value
        j = self.nowIdx
        self.nowIdx = (self.nowIdx+1)%self.stackSize
        self.dSum -= self.detErrorStack[j]
        self.iSum -= self.errorStack[j]
        self.detErrorStack[j] = nowError - self.preError
        self.errorStack[j] = nowError
        self.dSum += nowError - self.preError
        self.iSum += nowError
        
        output = self.p*nowError + self.i*self.iSum/self.stackSize + self.d*self.dSum/self.stackSize
        return output
    
    def setTarget(self, target:float):
        self.tarValue = target
class Car:
    def __init__(self, x:int=0, y:int=0):
        self.mass = 12
        self.frictionCoefficient = 0.7
        self.rollingfrictionCoefficient = 0.12
        self.wheelRad = 0.04
        self.maxOutTorque = 3
        self.maxOutPower = 30
        
        self.accNoiseRate = 2.2
        self.posNoiseRate = 4
        
        self.maxFriction = self.mass * 9.8 * self.frictionCoefficient # 82.32
        self.maxRollFriction = self.mass * 9.8 * self.rollingfrictionCoefficient # 14.112
        self.maxOutF = self.maxOutTorque / self.wheelRad
        
        self.pos = Vector2D(x, y)
        self.vel = Vector2D()
        self.acc = Vector2D()
        self.torque = Vector2D()
        
    def __stepImpl__(self, t:float=0):
        maxF = self.maxOutPower / (self.vel.rho() + 1e-5)
        velAlpha = self.vel.alpha()
        frac = Vector2D(math.cos(velAlpha), math.sin(velAlpha)) * self.maxRollFriction
        
        nowF = self.torque / self.wheelRad
        nowF.rhoLimit(maxF)
        
        realF = nowF - frac
        self.acc = addNoise(realF/self.mass, self.accNoiseRate, 0.2*t, 0.3)
        self.pos = self.pos + addNoise(self.vel*t + self.acc*t*t*0.5, self.posNoiseRate, 0.4*t, 0.3)
        self.vel = self.vel + self.acc*t

    def step(self, t:float=0):
        maxT = 0.0001
        while t > 0:
            self.__stepImpl__(min(maxT, t)) 
            t -= maxT
            
    def control(self, xTorque:float=0, yTorque:float=0):
        xTorque = clamp(xTorque, -1, 1)
        yTorque = clamp(yTorque, -1, 1)
        tarTorque = Vector2D(xTorque, yTorque)
        tarTorque.rhoLimit(1.0)
        self.torque = tarTorque * self.maxOutTorque
        
    def measureVelocity(self):
        velNoiseRate = 0.05
        velNoiseReal = 0.02
        return addNoise(self.vel, velNoiseRate, velNoiseReal)


class CarPID:
    def __init__(self):
        self.pidX = PID(p=10, i=2.5, d=1.8, stackSize=5)
        self.pidY = PID(p=10, i=2.5, d=1.8, stackSize=5)
    def measure(self, vel:Vector2D):
        x = self.pidX.measure(vel.x)
        y = self.pidY.measure(vel.y)
        return Vector2D(x, y)
    def setTarget(self, vel:Vector2D):
        self.pidX.setTarget(vel.x)
        self.pidY.setTarget(vel.y)
        
class Obstacle:
    def __init__(self, line:(Vector2D), rad:float=0.25, speed:float=0.6):
        self.rad = rad
        self.line = line
        self.speed = speed
        self.size = len(line)
        
        self.nowPos = line[0]
        self.nowIdx = 0
        
    def step(self, t:float):
        dis = self.speed*t
        if self.size != 1:
            i = (self.nowIdx+1)%self.size
            tar = self.line[i]
            while dis > 0:
                if (tar-self.nowPos).rho() >= dis:
                    self.nowPos = self.nowPos + (tar-self.nowPos).norm()*dis
                    dis = 0
                else:
                    dis -= (tar-self.nowPos).rho()
                    self.nowPos = tar
                    self.nowIdx = i
                    i = (i+1)%self.size
                    tar = self.line[i]


if __name__ == '__main__':
    TARGET_VEL = 1  # 期望速度
    MAX_ACC = 3
    TickTime = 0.001
    PIDControlTick = 50
    velocitySensorNoiseRate = 0.06
    velocitySensorNoiseReal = 0.02
    accControlNoiseRate = 0.06
    accControlNoiseReal = 0.02
    
    nowVelocity = 0
    nowAccel = 0
    pid = PID(p=10, i=2.5, d=1.8, stackSize=5)
    pidOutAcc = 0

    
    actVel, senVel, tarVel = [],[],[]
    actAcc, pidAcc = [],[]
    for i in range(10000):
        # targetVel = TARGET_VEL
        targetVel = (math.sin(i*TickTime)+1) * TARGET_VEL
        pid.setTarget(targetVel)
        velocitySensor = addNoise(nowVelocity, velocitySensorNoiseRate, velocitySensorNoiseReal)
        if i % PIDControlTick == 0 :
            pidOutAcc = pid.measure(velocitySensor)
        
        nowAccelAdd = clamp(minV=-MAX_ACC, maxV=MAX_ACC, x=addNoise(pidOutAcc, accControlNoiseRate, accControlNoiseReal))
        nowVelocity = nowVelocity + nowAccelAdd * TickTime
        if i < 3000:
            nowVelocity = min(nowVelocity, 0.5)
        
        tarVel.append(targetVel)
        actVel.append(nowVelocity)
        actAcc.append(nowAccelAdd)
        pidAcc.append(pidOutAcc)
        senVel.append(velocitySensor)
    
    fig, axs = plt.subplots(3, 1)
    axs[0].plot(actVel, label='actVel')
    # axs[0].plot(senVel, label='senVel')
    axs[0].plot(tarVel, label='tarVel')
    axs[0].set_title('Velocity')
    axs[0].set_xlabel('tick')
    axs[0].set_ylabel('velocity')
    axs[0].legend()
    
    # axs[1].plot(actVel, label='actVel')
    axs[1].plot(senVel, label='senVel')
    axs[1].plot(tarVel, label='tarVel')
    axs[1].set_title('Velocity')
    axs[1].set_xlabel('tick')
    axs[1].set_ylabel('velocity')
    axs[1].legend()
    
    axs[2].plot(actAcc, label='actAcc', linewidth=2)
    axs[2].plot(pidAcc, label='pidAcc')
    axs[2].set_title('Accel')
    axs[2].set_xlabel('tick')
    axs[2].set_ylabel('acc')
    axs[2].legend()
    
    plt.show()
