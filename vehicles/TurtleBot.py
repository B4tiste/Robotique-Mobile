#!/usr/bin/env python  

__author__ = 'Raphael Leber'

from math import cos, sin, pi, sqrt, atan2, fmod, fabs, copysign
import turtle  
import time

from vehicles.Vehicle import Vehicle
from PathTools import PathTools


class TurtleBot(Vehicle):

    def __init__(self, x=0, y=0, theta=0, Ks=5, Kv=2, R=0.1, L=0.2, dt=0.003):
        self.x = x          # x position
        self.y = y          # y position
        self.theta = theta  # theta orientation
        self.Ks = Ks        # Steering proportionnal coefficient
        self.Kv = Kv        # Velocity proportionnal coefficient
        self.R = R          # Radius of the wheel
        self.L = L          # Half the distance between the two wheels
        self.dt = dt        # Time of one code cycle
        self.throttle_min = 450    # Min throttle
        self.throttle_max = 700  # Max throttle

    def model(self, v_left, v_right):
        """  TurtleBot model
                           ___________________
                          |                   |
                          |                   |---> x           
            v_left    --->|                   |    
                          |     TurtleBot     |
                          |                   |---> y
                          |       Model       |                        
            v_right   --->|                   |   
                          |                   |---> theta      
                          |___________________|          
        """
        w = (v_right - v_left) / (2 * self.L)  # Angular velocity
        v = (v_right + v_left) / 2            # Linear velocity

        self.x += v * cos(self.theta) * self.dt
        self.y += v * sin(self.theta) * self.dt
        self.theta += w * self.dt

        return self.x, self.y, self.theta

    def toPoint(self, x, y, eps=0.5):
        """ Move to a point (x, y)
            Parameters:
            x (int): x coordinate to reach in the map
            y (int): y coordinate to reach in the map
            eps (float): epsilon below which we consider the goal is reached

            Returns:
            float: Returning throttle value (corresponding to euclidian distance vehicle-->goal) 
        """  
        Ks = self.Ks
        Kv = self.Kv          
        eucli_throttle = eps

        while eucli_throttle >= eps:
                
                eucli_throttle = sqrt((x - self.x)**2 + (y - self.y)**2)
    
                theta = atan2(y - self.y, x - self.x)
                s = PathTools().shortestAngleDiff(theta, self.theta) 

                omega = 2 * s * Ks
                base_speed = 100
                v_left = base_speed + (Kv * eucli_throttle) - (self.L * omega)
                v_right = base_speed + (Kv * eucli_throttle) + (self.L * omega)

                print(f"V_left: {v_left}, V_right: {v_right}")
    
                self.model(v_left, v_right)
    
                yield eucli_throttle

    def turn(self, phi, eps_angle=0.1):

        Ks = self.Ks
        s = 1

        while fabs(s) > eps_angle:

            s = PathTools().shortestAngleDiff(phi, self.theta) 

            omega = 2 * s * Ks
            v_left = -self.L * omega
            v_right = self.L * omega

            self.model(v_left, v_right)

            yield s            

    def toPose(self, x, y, theta):

        pass
