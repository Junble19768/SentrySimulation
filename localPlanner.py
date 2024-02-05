import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import time
from typing import Callable
from PID import *
from util import *

class Choice:
    def __init__(self, size:int):
        self.size = size
        self.line = (Vector2D,)*size
        self.acc = Vector2D()
        self.vel = Vector2D()
    

class LocalPlanner:
    def __init__(self, distancer:Callable[[Vector2D], tuple[float, Vector2D]], 
                 selfRad:float, 
                 maxAcc:float=2.0, 
                 maxVel:float=4.0):
        self.nowPos = Vector2D()
        self.tarPos = self.nowPos
        self.nowVel = Vector2D()
        self.maxAcc = maxAcc
        self.maxVel = maxVel
        self.selfRad = selfRad
        self.distancer = distancer
        
        # self.accRate = (0, 0.03, 0.07, 0.13, 0.20, 0.3, 0.4, 0.5, 0.6, 0.72, 0.85, 1) # 12
        self.accRate = (0, 0.05, 0.1, 0.2, 0.35, 0.5, 0.75, 1)
        self.timePoint = (0.05, 0.1, 0.15, 0.25, 0.3, 0.5) # 6
        self.angularSize = 18 # 36 
        
        self.timePointSize = len(self.timePoint)
        self.accRateSize = len(self.accRate)
        self.groupSize = self.angularSize*self.accRateSize
        self.angular = [-math.pi+(math.pi*2/self.angularSize)*i for i in range(0, self.angularSize)]
        
        
    def updateStatus(self, nowPos:Vector2D, nowVel:Vector2D):
        self.nowPos = nowPos
        self.nowVel = nowVel
    
    def setTargetPos(self, tarPos:Vector2D):
        self.tarPos = tarPos
        
        
    def disPenalty(self, dis:float, gradiant:Vector2D, nowVel:Vector2D):
        dis = max(dis, 1e-6)
        towardsVel = nowVel.dot(gradiant)
        
        dangerTimePenalty = 0
        if towardsVel <= 0:
            dangerTimePenalty = 0
        else :
            dangerTime = dis/(towardsVel+1e-4)
            if dangerTime < 1:
                dangerTimePenalty = 15/dangerTime # need a gradiant to show difference
            elif dangerTime < 2:
                dangerTimePenalty = 2/(dangerTime/2)
            elif dangerTime < 4:
                dangerTimePenalty = 1/dangerTime
            else :
                dangerTimePenalty = 0
        
        distancePenalty = 0
        if dis/self.selfRad < 0.5:
            distancePenalty = 4/(dis/self.selfRad)
        elif dis/self.selfRad < 1.2:
            distancePenalty = 2/(dis/self.selfRad)
        elif dis/self.selfRad < 2:
            distancePenalty = 1/(dis/self.selfRad)
        elif dis/self.selfRad < 3:
            distancePenalty = 0.8/(dis/self.selfRad)
        else:
            distancePenalty = 0

        return dangerTimePenalty + distancePenalty
    
    
    def score(self, groups:tuple[Choice,...]):
        groupSize = len(groups)
        groupScores = [0.0,]*groupSize
        tempScore = [0.0,]*groupSize
        
        for i,choice in enumerate(groups):
            tempScore[i] = 0
            for j,pos in enumerate(choice.line):
                dis, obPos = self.distancer(pos)
                gradiant = (obPos-pos).normalize()
                velocity = self.nowVel + choice.acc*self.timePoint[j]
                tempScore[i] -= self.disPenalty(dis, gradiant, velocity)
        # standardize(tempScore)
        for i in range(0, groupSize):
            groupScores[i] += tempScore[i]
        
        tarApproachValue = 15
        for i,choice in enumerate(groups):
            tempScore[i] = 0
            for pos in choice.line:
                tempScore[i] -= (self.tarPos-pos).len()
        tempScore = standardize(tempScore)
        for i in range(0, groupSize):
            groupScores[i] += tarApproachValue * tempScore[i]
        
        accAngularValue = 0.5
        for i,choice in enumerate(groups):
            tempScore[i] = 0
            acc = choice.acc
            vel = choice.vel
            tempScore[i] = acc.dot(vel)
        tempScore = standardize(tempScore)
        for i in range(0, groupSize):
            groupScores[i] += accAngularValue * tempScore[i]
        
        advanceSpeedValue = 2
        for i,choice in enumerate(groups):
            tempScore[i] = 0
            for t in self.timePoint:
                vel = self.nowVel + choice.acc*t
                tempScore[i] += vel.dot(self.tarPos-self.nowPos)
        tempScore = standardize(tempScore)
        for i in range(0, groupSize):
            groupScores[i] += advanceSpeedValue * tempScore[i]
        
        return tuple(groupScores)
    
    def plan(self):
        groups = [Choice(self.timePointSize) for _ in range(self.accRateSize*self.angularSize)]
        
        # generate line
        nowIdx = 0
        for angular in self.angular:
            for accRate in self.accRate:
                acc = Vector2D(math.cos(angular), math.sin(angular))*self.maxAcc*accRate
                groups[nowIdx].acc = acc
                groups[nowIdx].vel = self.nowVel
                
                tempLine = []
                for i,t in enumerate(self.timePoint):
                    nowPos = self.nowPos + self.nowVel*t + acc*(t**2)*0.5
                    tempLine.append(nowPos)
                groups[nowIdx].line = tuple(tempLine)
                nowIdx += 1

        groups = tuple(groups)
        scores = self.score(groups)
        nowScore = -1.145e4
        nowIdx = -1
        for i in range(0, len(groups)):
            if scores[i] > nowScore:
                nowScore = scores[i]
                nowIdx = i
        tarSpeed = groups[nowIdx].vel + groups[nowIdx].acc*0.5
        tarVec = self.tarPos-self.nowPos
        if tarVec.len() < self.selfRad:
            tarSpeed = tarVec
        if tarVec.len() < self.selfRad * 0.3:
            tarSpeed = Vector2D(0, 0)
        return nowIdx, groups, tarSpeed
    