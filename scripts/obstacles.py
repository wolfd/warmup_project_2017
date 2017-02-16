#!/usr/bin/env python2

import math

class Goal(object):
    """Reperesents a something the robot is attracted to"""
    def __init__(self, x, y, radius, attraction_limit, attraction_factor):
        super(Goal, self).__init__()

        self.x = x # X position of the goal
        self.y = y # Y position of the goal
        self.radius = radius # The radius of the goal (represented as a sphere)
        self.attraction_limit = attraction_limit # The limit at which the attractive force stops getting larger
        self.attraction_factor = attraction_factor # Proportional to the strength of the attractive force


    def get_distance(self, x, y):
        """
        Computes the distance between a point and the goal

        Args:
            x: x position of point we are calculating distance to
            y: y position of point we are calculating distance to
        """
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    def get_mag(self, x, y):
        """
        Computes the magnitude of the attractive force at a given point

        Args:
            x: x position of point we are calculating attractive force for
            y: y position of point we are calculating attractive force for
        """

        # Get distance to a point
        d = self.get_distance(x, y)
        # If within goal, the attractive force is 0
        if d < self.radius:
            return 0
        # Up to the limit, the attractive force is proportional to d
        if d >= self.radius and (self.attraction_limit+self.radius) >= d:
            return self.attraction_factor * (d - self.radius)
        # Past that, the attractive force is constant
        else:
            return self.attraction_factor * self.attraction_limit

    def get_angle(self, x, y):
        """
        Computes the angle between the goal and a given point

        Args:
            x: x position of point we are calculating the angle to
            y: y position of point we are calculating the angle to
        """

        theta = math.atan2(self.y - y, self.x- x)
        return theta

class Obstacle(object):
    """Represents something the robot is repelled by"""
    def __init__(self, x, y, radius, repellant_limit, repellant_factor):
        super(Obstacle, self).__init__()
        self.x = x # X position of the obstacle
        self.y = y # Y position of the obstacle
        self.radius = radius # The radius of the obstacle (represented as a sphere)
        self.repellant_limit = repellant_limit # The limit at which the obstacle stops repelling things
        self.repellant_factor = repellant_factor # Proportional to the strength of the attractive force

    def get_mag(self, x, y):
        """
        Computes the magnitude of the repellant force at a given point

        Args:
            x: x position of point we are calculating repellant force for
            y: y position of point we are calculating repellant force for
        """
        # Calculate distance to the point
        d = math.sqrt((self.x - x)**2 + (self.y - y)**2)
        # If we're inside the obstacle
        if d < self.radius:
            # Move really fast out
            return -5
        # Within the limit the repellant force is proportional to the distance
        if d >= self.radius and (self.repellant_limit+self.radius) >= d:
             return -self.repellant_factor*(self.repellant_limit + self.radius - d)
        # If outside the limit, there is no repellant force
        else:
            return 0

    def get_angle(self, x, y):
        """
        Computes the angle between the obstacle and a given point

        Args:
            x: x position of point we are calculating the angle to
            y: y position of point we are calculating the angle to
        """
        theta = math.atan2(self.y - y, self.x- x)
        return theta
