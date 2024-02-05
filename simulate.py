import pygame
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import time
from PID import *
from util import *
from localPlanner import *
import os
        
class Simulate:
    def __init__(self):
        self.simSize = (10, 6)
        self.perPixel = 0.01
        self.winSize = (int(self.simSize[0]/self.perPixel), int(self.simSize[1]/self.perPixel)) # 设置窗口大小
        
        self.sentry = Car(1, 1)
        self.sentryRad = 0.2
        self.sentryColor = (233, 57, 125)
        self.sentryPID = CarPID()
        self.sentryPast = []
        
        self.obstancles = []
        self.obstancles.append(Obstacle(line=(Vector2D(2,2),Vector2D(2,5.5),Vector2D(5.5,5.5),Vector2D(5.5,2)), speed=1))
        self.obstancles.append(Obstacle(line=(Vector2D(6,2),Vector2D(4,4)), rad=0.7, speed=0.3))
        for _ in range(8):
            self.obstancles.append(RandObstacle(self.simSize, speed=0.7))
        for _ in range(4):
            self.obstancles.append(RandObstacle(self.simSize, speed=0.3))
        for _ in range(3):
            self.obstancles.append(RandObstacle(self.simSize, speed=1.2))
        
        self.nowT = 0
        self.FPS = 60
        self.obstancleHZ = 30
        self.sentryPIDHZ = 60
        self.sentryUpdateHZ = 120
        self.sentryControlHZ = 10
        self.timeQue = TimeCallQue(-0.1)
        self.localPlanner = LocalPlanner(self.getDis, self.sentryRad, maxAcc=4, maxVel=3)
        self.choiceGroup = (Choice(self.localPlanner.timePointSize),)*self.localPlanner.groupSize
        self.choiceIdx = 0
        self.timeQue.add(0, self.stepObstancle)
        self.timeQue.add(0, self.stepSentryUpdate)
        self.timeQue.add(0, self.stepSentryPID)
        self.timeQue.add(0, self.stepSentryControl)
        pygame.init()
        self.screen = pygame.display.set_mode(self.winSize) # 创建窗口
        pygame.display.set_caption('Sentry Simulation')
        
        self.font = pygame.font.Font(None, 20)
        
        self.sentryGoal = self.sentry.pos
    
    def Real2Screen(self, x:float, y:float):
        x = clamp(int(x/self.perPixel), 0, self.winSize[0])
        y = clamp(int(y/self.perPixel), 0, self.winSize[1])
        return (x, y)

    def Screen2Real(self, x:int, y:int):
        return (x*self.perPixel, y*self.perPixel)

    
    def stepObstancle(self, nowT):
        time = 1/self.obstancleHZ
        self.timeQue.add(nowT+time, self.stepObstancle)
        for ob in self.obstancles:
            ob.step(time)
        
        
    def stepSentryControl(self, nowT):
        time = 1/self.sentryControlHZ
        self.timeQue.add(nowT+time, self.stepSentryControl)
        self.localPlanner.updateStatus(self.sentry.measurePos(), self.sentry.measureVelocity())
        self.localPlanner.setTargetPos(self.sentryGoal)
        self.choiceIdx, self.choiceGroup, tarVel = self.localPlanner.plan()
        self.sentryPID.setTarget(tarVel)
        # os.system("pause")
        
 
    def stepSentryPID(self, nowT):
        time = 1/self.sentryPIDHZ
        self.timeQue.add(nowT+time, self.stepSentryPID)
        pidOut = self.sentryPID.measure(self.sentry.measureVelocity())
        self.sentry.control(pidOut.x, pidOut.y)
        
    
    def stepSentryUpdate(self, nowT):
        time = 1/self.sentryUpdateHZ
        self.timeQue.add(nowT+time, self.stepSentryUpdate)
        self.sentry.step(time)
        
        
    def getDis(self, point:Vector2D):
        minD = 1.14e51
        obPos = Vector2D()
        for ob in self.obstancles:
            dis = (ob.nowPos-point).len()-ob.rad
            if dis < minD :
                minD = dis
                obPos = ob.nowPos
        if abs(point.x-0) < minD:
            minD = abs(point.x-0)
            obPos = Vector2D(0, point.y)
        if abs(point.x-self.simSize[0]) < minD:
            minD = abs(point.x-self.simSize[0])
            obPos = Vector2D(self.simSize[0], point.y)
        if abs(point.y-0) < minD:
            minD = abs(point.y-0)
            obPos = Vector2D(point.x, 0)
        if abs(point.y-self.simSize[1]) < minD:
            minD = abs(point.y-self.simSize[1])
            obPos = Vector2D(point.x, self.simSize[1])
        return minD, obPos
        
    def eventHandler(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.sentryGoal = Vector2D(*self.Screen2Real(*event.pos))
        
    def draw(self):
        self.screen.fill((38, 38, 38))  # 清除屏幕
        
        # draw obstancle
        color = (255, 215, 14)
        for ob in self.obstancles:
            nowPos = self.Real2Screen(ob.nowPos.x, ob.nowPos.y)
            pygame.draw.circle(self.screen, color, nowPos, int(ob.rad/self.perPixel))
            
        # draw sentry
        if self.sentry.pos.x >= self.simSize[0]:
            self.sentry.pos.x = 0 + 1e-4
        elif self.sentry.pos.x <= 0:
            self.sentry.pos.x = self.simSize[0] - 1e-4
        if self.sentry.pos.y >= self.simSize[1]:
            self.sentry.pos.y = 0 + 1e-4
        elif self.sentry.pos.y <= 0:
            self.sentry.pos.y = self.simSize[1] - 1e-4
        nowPos = self.Real2Screen(self.sentry.pos.x, self.sentry.pos.y)
        self.sentryPast.append((nowPos[0], nowPos[1]))
        pygame.draw.circle(self.screen, self.sentryColor, nowPos, int(self.sentryRad/self.perPixel))
        
        # draw sentry path
        if len(self.sentryPast) > 1: # 画轨迹线
            pygame.draw.lines(self.screen, (144, 144, 144), False, self.sentryPast, 2)
        
        for i,choice in enumerate(self.choiceGroup):
            if i == self.choiceIdx:
                continue
            temp = []
            for pos in choice.line:
                nowPos = self.Real2Screen(pos.x, pos.y)
                temp.append((nowPos[0], nowPos[1]))
            pygame.draw.lines(self.screen, (200, 200, 200), False, temp, 1)
        choice = self.choiceGroup[self.choiceIdx]
        temp = []
        for pos in choice.line:
            nowPos = self.Real2Screen(pos.x, pos.y)
            temp.append((nowPos[0], nowPos[1]))
        pygame.draw.lines(self.screen, (255, 0, 0), False, temp, 3)
        
        # draw sentry goal
        nowPos = self.Real2Screen(self.sentryGoal.x, self.sentryGoal.y)
        pygame.draw.circle(self.screen, (255,0,0), nowPos, 6)
        
        # draw text
        dis, _ = self.getDis(self.sentry.pos)
        text = self.font.render(f'dis: ({dis-self.sentryRad:.2f})', True, (255, 255, 255))
        self.screen.blit(text, (self.winSize[0] - 150, 15))
        text = self.font.render(f'Goal: ({self.sentryGoal.x:.2f}  {self.sentryGoal.y:.2f})', True, (255, 255, 255))
        self.screen.blit(text, (self.winSize[0] - 150, 30))
        
        pygame.display.flip() # 刷新屏幕
        
    def run(self):
        while True:
            self.eventHandler()
            self.timeQue.step(self.nowT)
            self.draw()
            pygame.time.Clock().tick(self.FPS) # 控制帧率
            self.nowT += 1/self.FPS