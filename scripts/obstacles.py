#!/usr/bin/env python2

import math

class Goal(object):
    """Reperesents a something the robot is attracted to"""
    def __init__(self, x, y, radius, limit, alpha):
        super(Goal, self).__init__()
        self.x = x
        self.y = y
        self.radius = radius
        self.limit = limit
        self.alpha = alpha

    def get_distance(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    def get_mag(self, x, y):
        d = self.get_distance(x, y)
        if d < self.radius:
            return 0
        if d >= self.radius and (self.limit+self.radius) >= d:
            return self.alpha * (d - self.radius)
        else:
            return self.alpha * self.limit

    def get_angle(self, x, y):
        theta = math.atan2(self.y - y, self.x- x)
        return theta

class Obstacle(object):
    """Represents something the robot is repelled by"""
    def __init__(self, x, y, radius, limit, beta):
        super(Obstacle, self).__init__()
        self.x = x
        self.y = y
        self.radius = radius
        self.limit = limit
        self.beta = beta

    def get_mag(self, x, y):
        d = math.sqrt((self.x - x)**2 + (self.y - y)**2)
        if d < self.radius:
            # if inside of obstacle move really fast out
            return -5
        if d >= self.radius and (self.limit+self.radius) >= d:
             return -self.beta*(self.limit + self.radius - d)
        else:
            return 0

    def get_angle(self, x, y):
        theta = math.atan2(self.y - y, self.x- x)
        return theta
