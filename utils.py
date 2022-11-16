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
        if xPointB == xPointA:
            xPointA += 0.001
        a = (yPointB - yPointA) / (xPointB - xPointA)
        b = yPointA - a * xPointA
        return a, b

    @staticmethod
    def findSolOfEquation(a, b, c):
        # aX^2 + bX + c = 0
        delta = b**2 - 4*a*c
        # print("delta: ", delta)
        if delta < 0:
            return 0, 0, 0
        if delta == 0:
            return 1, -b/(2*a), -b/(2*a)
        else:
            return 2, (-b + math.sqrt(delta))/(2*a), (-b - math.sqrt(delta))/(2*a)
    

    @staticmethod
    def getDistanceFromObstacle(xCenter, yCenter, xTarget, yTarget, xObstacle, yObstacle):
        a, b = Utils.findLinePassTwoPoints(xCenter, yCenter, xTarget, yTarget)
        a_temp = a**2 + 1
        b_temp = -2*xObstacle + 2*a*(b - yObstacle)
        c_temp = (b - yObstacle)**2 + xObstacle**2 - PlayerParam.RADIUS_OBJECT**2
        numberOfSolution, x1, x2 = Utils.findSolOfEquation(a_temp, b_temp, c_temp)
        
        
        if numberOfSolution == 0:
            return PlayerParam.INFINITY
        elif numberOfSolution == 1:
            return Utils.distanceBetweenTwoPoints(xCenter, yCenter, x1, a*x1 + b)
        else:
            if abs(xCenter - x1) < abs(xCenter - x2):
                return Utils.distanceBetweenTwoPoints(xCenter, yCenter, x1, a*x1 + b)
            else:
                return Utils.distanceBetweenTwoPoints(xCenter, yCenter, x2, a*x2 + b)
            
    