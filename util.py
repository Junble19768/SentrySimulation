import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import heapq

def clamp(x, minV, maxV):
    return min(maxV, max(minV, x))

def toPolar(x, y):
    return (math.atan2(y, x), math.sqrt(x**2 + y**2))

def toCartesian(alpha, rho):
    return (rho*math.cos(alpha), rho*math.sin(alpha))



class Vector2D:
    def __init__(self, x:float=0, y:float=0):
        self.x = x
        self.y = y
    def __str__(self):
        return f"({self.x:.2f}, {self.y:.2f})"
    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x + other.x, self.y + other.y)
        elif isinstance(other, (int, float)):
            return Vector2D(self.x + other, self.y + other)
        raise ValueError("Unsupported operand type for +")
    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x - other.x, self.y - other.y)
        elif isinstance(other, (int, float)):
            return Vector2D(self.x - other, self.y - other)
        raise ValueError("Unsupported operand type for -")
    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Vector2D(self.x*other, self.y*other)
        raise ValueError("Unsupported operand type for *")
    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            if other == 0:
                raise ValueError("Division by zero")
            return Vector2D(self.x/other, self.y/other)
        raise ValueError("Unsupported operand type for /")
    def dot(self, other):
        if isinstance(other, Vector2D):
            return self.x * other.x + self.y * other.y
        raise ValueError("Unsupported operand type for dot product")
    def len(self):
        return self.rho()
    def rho(self):
        return math.sqrt(self.x**2 + self.y**2)
    def alpha(self):
        return math.atan2(self.y, self.x)
    def normalize(self):
        return self/self.rho()
    def rhoLimit(self, value:float):
        alpha, rho = toPolar(self.x, self.y)
        rho = min(value, rho)
        self.x, self.y = toCartesian(alpha, rho)
        
def addNoise(x, noiseRate:float=0.08, realNoiseLen:float=0, rate:float=1):
    if random.uniform(0,1) <= rate:
        if isinstance(x, (int, float)):
            return x + random.normalvariate(0, 1)*x*noiseRate + random.normalvariate(0, 1)*realNoiseLen
        if isinstance(x, Vector2D):
            drho = random.uniform(0, 1)*x.rho()*noiseRate + random.uniform(0, 1)*realNoiseLen
            dalpha = random.uniform(0, math.pi*2)
            return x + Vector2D(*toCartesian(dalpha, drho))
    return x
    
class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._index = 0
    def push(self, item, priority):
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1
    def pop(self):
        return heapq.heappop(self._queue)[-1]
    def top(self):
        topItem = heapq.nsmallest(1, self._queue)[0]
        return topItem[0], topItem[-1]
    def size(self):
        return len(self._queue)

class TimeCallQue:
    def __init__(self, nowTime:float=0):
        self.que = PriorityQueue()
        self.nowTime = nowTime
    def add(self, time, callback):
        if time<self.nowTime:
            raise RuntimeError('Que: pushing a past time!')
        self.que.push(callback, time)
    def step(self, tarTime):
        while self.que.size() > 0 :
            firstT,_ = self.que.top()
            if firstT <= tarTime :
                self.nowTime = firstT
                callback = self.que.pop()
                callback(self.nowTime)
            else:
                break
        self.nowTime = tarTime
        
def standardize(inputArray:list[float]):
    num = np.array(inputArray, dtype=float) # 将元组转换为 NumPy 数组
    avg = np.mean(num) # 计算均值和标准差
    stdDeviation = np.std(num)
    standardized = (num - avg) / stdDeviation # 标准化处理
    return list(standardized)

# def reContribute(inputArray:list[float]):
    