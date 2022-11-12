import pygame
import math
from const import *

class Utils:
    @staticmethod
    def blit_rotate_center(win, image, top_left, angle):
        rotated_image = pygame.transform.rotate(image, angle)
        new_rect = rotated_image.get_rect(
            center=image.get_rect(topleft=top_left).center)
        win.blit(rotated_image, new_rect.topleft)
        
    @staticmethod
    def distanceBetweenTwoPoints(xPointA, yPointA, xPointB, yPointB):
        return math.sqrt((xPointA - xPointB)**2 + (yPointA - yPointB)**2)
    
    @staticmethod
    def findLinePassTwoPoints(xPointA, yPointA, xPointB, yPointB):
        # y = ax + b
        a = (yPointB - yPointA) / (xPointB - xPointA)
        b = yPointA - a * xPointA
        return a, b

    @staticmethod
    def findSolOfEquation(a, b, c):
        # aX^2 + bX + c = 0
        delta = b**2 - 4*a*c
        if delta < 0:
            return Equation.NO_SOLUTION, 0, 0
        if delta == 0:
            return Equation.ONE_SOLUTION, -b/(2*a), -b/(2*a)
        else:
            return Equation.TWO_SOLUTION, (-b + math.sqrt(delta))/(2*a), (-b - math.sqrt(delta))/(2*a)
    
    @staticmethod
    def getDistanceFromObstacle(xCenter, yCenter, xTarget, yTarget, xObstacle, yObstacle):
        # Pt đường thẳng lidar y = ax + b
        a, b = Utils.findLinePassTwoPoints(xCenter, yCenter, xTarget, yTarget)
        # Pt đường thẳng cắt hình tròn (a^2 + 1)x^2 - 2*(2*xCenter - ab + yCenter)x + (b - yCenter)**2 - RADIUS_LIDAR = 0
        a_temp = a**2 + 1
        b_temp = -2*(a*xObstacle - a*b + yObstacle)
        c_temp = (b - yObstacle)**2 - PlayerParam.RADIUS_LIDAR**2
        numberOfSolution, x1, x2 = Utils.findSolOfEquation(a_temp, b_temp, c_temp)
        
        if numberOfSolution == Equation.NO_SOLUTION:
            print('NO_SOLUTION')
            return PlayerParam.INFINITY
        elif numberOfSolution == Equation.ONE_SOLUTION:
            return x1
        else:
            d1 = Utils.distanceBetweenTwoPoints(xCenter, yCenter, x1, a*x1 + b)
            d2 = Utils.distanceBetweenTwoPoints(xCenter, yCenter, x2, a*x2 + b)
            return x1 if x1 <= x2 else x2