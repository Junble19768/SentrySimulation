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
        
class Simulate:
    def __init__(self):
        self.simSize = (10, 6)
        self.perPixel = 0.01
        self.winSize = (int(self.simSize[0]/self.perPixel), int(self.simSize[1]/self.perPixel)) # 设置窗口大小
        
        self.sentry = Car(1, 1)
        self.sentryRad = 0.2
        self.sentryColor = (233, 57, 125)
        self.sentryControl = CarControl()
        self.sentryPast = []
        
        self.obstancles = []
        self.obstancles.append(Obstacle(line=(Vector2D(2,2),Vector2D(2,5.5),Vector2D(5.5,5.5),Vector2D(5.5,2)), speed=1))
        self.obstancles.append(Obstacle(line=(Vector2D(6,2),Vector2D(4,4)), rad=0.7, speed=0.3))
        
        self.nowT = 0
        self.FPS = 60
        self.obstancleHZ = 30
        self.sentryPIDHZ = 30
        self.sentryUpdateHZ = 60
        self.timeQue = TimeCallQue(-0.1)
        self.timeQue.add(0, self.stepObstancle)
        self.timeQue.add(0, self.stepSentryUpdate)
        self.timeQue.add(0, self.stepSentryPID)
        pygame.init()
        self.screen = pygame.display.set_mode(self.winSize) # 创建窗口
        pygame.display.set_caption('Sentry Simulation')
    
    def Real2Screen(self, x:float, y:float):
        x = clamp(int(x/self.perPixel), 0, self.winSize[0])
        y = clamp(int(y/self.perPixel), 0, self.winSize[1])
        return (x, y)

    def drawSentry(self):
        if self.sentry.pos.x >= self.simSize[0]:
            self.sentry.pos.x = 0
        elif self.sentry.pos.x <= 0:
            self.sentry.pos.x = self.simSize[0]
        if self.sentry.pos.y >= self.simSize[1]:
            self.sentry.pos.y = 0
        elif self.sentry.pos.y <= 0:
            self.sentry.pos.y = self.simSize[1]
        nowPos = self.Real2Screen(self.sentry.pos.x, self.sentry.pos.y)
        self.sentryPast.append((nowPos[0], nowPos[1]))
        pygame.draw.circle(self.screen, self.sentryColor, nowPos, int(self.sentryRad/self.perPixel))
        if len(self.sentryPast) > 1: # 画轨迹线
            pygame.draw.lines(self.screen, (144, 144, 144), False, self.sentryPast, 2)
            
    def drawObstancle(self):
        color = (255, 215, 14)
        for ob in self.obstancles:
            nowPos = self.Real2Screen(ob.nowPos.x, ob.nowPos.y)
            pygame.draw.circle(self.screen, color, nowPos, int(ob.rad/self.perPixel))
    
    def stepObstancle(self, nowT):
        time = 1/self.obstancleHZ
        for ob in self.obstancles:
            ob.step(time)
        self.timeQue.add(nowT+time, self.stepObstancle)

    def stepSentryPID(self, nowT):
        time = 1/self.sentryPIDHZ
        tarVel = Vector2D(math.cos(nowT)*0.5, math.sin(nowT)*0.5)
        self.sentryControl.setTarget(tarVel)
        pidOut = self.sentryControl.measure(self.sentry.vel)
        self.sentry.control(pidOut.x, pidOut.y)
        self.timeQue.add(nowT+time, self.stepSentryPID)
    
    def stepSentryUpdate(self, nowT):
        time = 1/self.sentryUpdateHZ
        self.sentry.step(time)
        self.timeQue.add(nowT+time, self.stepSentryUpdate)
        
    def getDis(self, point:Vector2D):
        minD = 1.14e51
        for ob in self.obstancles:
            minD = min(minD, (ob.nowPos-point).rho()-ob.rad)
        return minD
        
    def run(self):
        # 加载显示坐标的字体
        font = pygame.font.Font(None, 36)
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            self.timeQue.step(self.nowT)
            
            self.screen.fill((38, 38, 38))  # 清除屏幕
            
            dis = self.getDis(self.sentry.pos)
            text = font.render(f'dis: ({dis-self.sentryRad:0.3f})', True, (255, 255, 255))
            self.screen.blit(text, (self.winSize[0] - 150, 20))
            
            self.drawSentry()
            self.drawObstancle()
            pygame.display.flip() # 刷新屏幕

            pygame.time.Clock().tick(self.FPS) # 控制帧率
            self.nowT += 1/self.FPS
            

if __name__ == '__main__':
    random.seed(time.time())
    game = Simulate()
    game.run()