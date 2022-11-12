
import math
import pygame
from sys import exit
import random
from const import *
from utils import *
from table import *
import datetime
import time

# from table import RLAlgorithm

globalX = 0

def getGlobalX():
    return globalX


class Player():
    def __init__(self, maxVelocity, maxRotationVelocity):
        super().__init__()
        global GLOBAL_SCREEN
        self.xPos, self.yPos = PlayerParam.INITIAL_X, PlayerParam.INITIAL_Y
        self.maxVelocity = maxVelocity
        self.maxRotationVelocity = maxRotationVelocity

        self.currVelocity = 0  # always >= 0
        self.currRotationVelocity = 0  # rotate left < 0, rotate right > 0
        self.currAngle = math.pi
        self.accelerationForward = PlayerParam.ACCELERATION_FORWARD
        if GameSettingParam.DRAW:
            self.circleRect = pygame.draw.circle(
                GLOBAL_SCREEN, CustomColor.RED, (self.xPos, self.yPos), PlayerParam.RADIUS_OBJECT)

        # Raycasting
        self.rayCastingLists = [PlayerParam.INFINITY] * PlayerParam.CASTED_RAYS

        self.mode = MODE_PLAY.MANUAL
        self.displayGUI = GUI.DISPLAY

    def _move(self):
        dt = float(1/GameSettingParam.FPS)

        self.yPos += math.cos(self.currAngle) * self.currVelocity * dt
        self.xPos += -math.sin(self.currAngle) * self.currVelocity * dt
        self.currAngle += self.currRotationVelocity*dt
        # print(math.degrees(self.currAngle%(math.pi*2)))

    def _playerInput(self, actionIndex):
        if (self.mode == MODE_PLAY.MANUAL):
            keys = pygame.key.get_pressed()



            # Rotate left ()
            if keys[pygame.K_a]:
                self.currRotationVelocity -= PlayerParam.ACCELERATION_ROTATE
            # Rotate right ()
            if keys[pygame.K_d]:
                self.currRotationVelocity += PlayerParam.ACCELERATION_ROTATE

            # Increase forward velocity
            if keys[pygame.K_w]:
                self.currVelocity = min(
                    self.currVelocity + PlayerParam.ACCELERATION_FORWARD, self.maxVelocity)

            # Stop
            if keys[pygame.K_s]:
                self.currVelocity = 0
                self.currRotationVelocity = 0

            # Decrease forward velocity
            if keys[pygame.K_x]:
                self.currVelocity = max(
                    self.currVelocity - PlayerParam.ACCELERATION_FORWARD, 0)

        elif (self.mode == MODE_PLAY.RL_TRAIN):

            if RLParam.ACTIONS[actionIndex] == PlayerParam.DESC_ROTATION_VELO:
                self.currRotationVelocity -= PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[actionIndex] == PlayerParam.INC_ROTATION_VELO:
                self.currRotationVelocity += PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[actionIndex] == PlayerParam.STOP:
                self.currVelocity = 0
                self.currRotationVelocity = 0

            if RLParam.ACTIONS[actionIndex] == PlayerParam.INC_FORWARD_VELO:
                self.currVelocity = min(
                    self.currVelocity + PlayerParam.ACCELERATION_FORWARD, self.maxVelocity)

            if RLParam.ACTIONS[actionIndex] == PlayerParam.DESC_FORWARD_VELO:
                self.currVelocity = max(
                    self.currVelocity - PlayerParam.ACCELERATION_FORWARD, 0)

    def _rayCasting(self):
        # print(math.degrees(self.currAngle),math.sin(self.currAngle))
        global obstacles
        inRangedObj = []

        # stop = False

        for obstacle in obstacles:
            

            # vector = (obstacle.xPos - self.xPos,obstacle.yPos - self.yPos)
            # standa = (math.sin(self.currAngle),math.cos(self.currAngle))
            # alpha = math.acos((vector[0]*standa[0] + vector[1]*standa[1])/((math.sqrt(vector[0]**2 + vector[1]**2))*math.sqrt(standa[0]**2 + standa[1]**2)))
            #   and math.degrees(alpha) < 90
            
            if Utils.distanceBetweenTwoPoints(self.xPos, self.yPos, obstacle.xPos, obstacle.yPos) < PlayerParam.RADIUS_LIDAR + PlayerParam.RADIUS_OBJECT:
                inRangedObj.append(obstacle)
                # print("alpha: ",alpha)
        startAngle = self.currAngle - PlayerParam.HALF_FOV
        # print(len(inRangedObj), " obg time: ",time.time() - startTime)
        
        # print("currr angel",math.degrees(self.currAngle))

        if len(inRangedObj) == 0:
            for ray in range(PlayerParam.CASTED_RAYS):
                if GameSettingParam.DRAW:
                    target_x = self.xPos - math.sin(startAngle) * PlayerParam.RADIUS_LIDAR
                    target_y = self.yPos + math.cos(startAngle) * PlayerParam.RADIUS_LIDAR
                    pygame.draw.line(GLOBAL_SCREEN, CustomColor.GREEN, (self.xPos, self.yPos), (target_x, target_y))
                self.rayCastingLists[ray] = PlayerParam.INFINITY
                startAngle += PlayerParam.STEP_ANGLE
            # print("none obj: ",time.time() - startTime)  
        else:
            for ray in range(PlayerParam.CASTED_RAYS):
                # get ray target coordinates
                isDetectObject = False
                
                target_x = self.xPos - math.sin(startAngle) * PlayerParam.RADIUS_LIDAR
                target_y = self.yPos + math.cos(startAngle) * PlayerParam.RADIUS_LIDAR

                for obstacle in inRangedObj:
                    theda = math.sqrt((obstacle.xPos - self.xPos)**2+(obstacle.yPos - self.yPos)**2)
                    beta = 0
                    if obstacle.xPos - self.xPos < 0:
                        beta = math.acos((obstacle.yPos - self.yPos)/theda) - startAngle
                    else:
                        beta = math.acos((obstacle.yPos - self.yPos)/theda) + startAngle
                    height = theda*math.sin(beta)
                    
                    tvh = (self.xPos-obstacle.xPos)*math.sin(startAngle) + (obstacle.yPos-self.yPos)*math.cos(startAngle)
                    if abs(height) < PlayerParam.RADIUS_OBJECT and tvh > 0:
                        isDetectObject = True

                if not isDetectObject:
                    if GameSettingParam.DRAW:
                        pygame.draw.line(GLOBAL_SCREEN, CustomColor.RED, (self.xPos, self.yPos), (target_x, target_y))
                    self.rayCastingLists[ray] = PlayerParam.INFINITY
                    startAngle += PlayerParam.STEP_ANGLE
                    continue

                isDetectObject = False

                for obstacle in inRangedObj:
                    distance = Utils.getDistanceFromObstacle(self.xPos, self.yPos, target_x, target_y, obstacle.xPos, obstacle.yPos)
                    self.rayCastingLists[ray] = distance
                    if distance <= PlayerParam.RADIUS_LIDAR:
                        # stop = True
                        isDetectObject = True
                        if GameSettingParam.DRAW:
                            pygame.draw.line(GLOBAL_SCREEN, CustomColor.WHITE, (self.xPos, self.yPos), (target_x, target_y))
                    else:
                        if GameSettingParam.DRAW:
                            pygame.draw.line(GLOBAL_SCREEN, CustomColor.PINK, (self.xPos, self.yPos), (target_x, target_y))
                    if isDetectObject:
                        break

                # for fdepth in range(int((PlayerParam.RADIUS_LIDAR)/10+1)):
                #     depth = fdepth*10
                #     target_x = self.xPos - \
                #         math.sin(startAngle) * depth
                #     target_y = self.yPos + \
                #         math.cos(startAngle) * depth

                #     # print(int(target_x),int(target_y),":",end="")

                #     for obstacle in inRangedObj:
                #         distance = Utils.distanceBetweenTwoPoints(
                #             target_x, target_y, obstacle.xPos, obstacle.yPos)
                #         if distance <= PlayerParam.RADIUS_OBJECT:
                #             self.rayCastingLists[ray] = Utils.distanceBetweenTwoPoints(
                #             target_x, target_y, self.xPos, self.yPos)
                #             # stop = True
                #             isDetectObject = True
                #             if GameSettingParam.DRAW:
                #                 pygame.draw.line(GLOBAL_SCREEN, CustomColor.WHITE, (self.xPos, self.yPos), (target_x, target_y))
                #         if depth == PlayerParam.RADIUS_LIDAR and not isDetectObject:
                #             self.rayCastingLists[ray] = PlayerParam.INFINITY
                #             if GameSettingParam.DRAW:
                #                 pygame.draw.line(GLOBAL_SCREEN, CustomColor.PINK, (self.xPos, self.yPos), (target_x, target_y))
                #         if isDetectObject:
                #             break
                        
                # print("\n\n")
                startAngle += PlayerParam.STEP_ANGLE
            # print("obj: ",time.time() - startTime)  
        # print("_rayCasting: ",time.time() - startTime)
        # print(self.rayCastingLists)
        # if (stop):
        #     time.sleep(1)

    def checkCollision(self):
        global obstacles
        for obstacle in obstacles:
            distanceBetweenCenter = Utils.distanceBetweenTwoPoints(
                self.xPos, self.yPos, obstacle.xPos, obstacle.yPos)
            # https://stackoverflow.com/questions/22135712/pygame-collision-detection-with-two-circles
            if distanceBetweenCenter <= 2*PlayerParam.RADIUS_OBJECT:
                # print("Ouch!!!",end="")
                return True
                # pass
        return False

    def draw(self, actionIndex):
        global GLOBAL_SCREEN
        self._playerInput(actionIndex=actionIndex)
        self._rayCasting()
        self.checkCollision()
        self._move()

        if GameSettingParam.DRAW:
            # draw player on 2D board
            pygame.draw.circle(GLOBAL_SCREEN, CustomColor.RED,
                               (self.xPos, self.yPos), PlayerParam.RADIUS_OBJECT)

            # draw player direction
            pygame.draw.line(GLOBAL_SCREEN, CustomColor.GREEN, (self.xPos, self.yPos),
                             (self.xPos - math.sin(self.currAngle) * 20,
                              self.yPos + math.cos(self.currAngle) * 20), 3)


class Obstacle(Player):
    def __init__(self):
        super().__init__(maxVelocity=PlayerParam.MAX_VELOCITY,
                         maxRotationVelocity=PlayerParam.MAX_ROTATION_VELOCITY)

        self.xPos = random.randint(int(0.3*GameSettingParam.WIDTH), int(
            0.7*GameSettingParam.WIDTH))

        self.yPos = ObstacleParam.INITIAL_OBSTACLE_Y + random.randint(0, int(
            0.9*GameSettingParam.HEIGHT))
        if GameSettingParam.DRAW:
            self.circleRect = pygame.draw.circle(
                GLOBAL_SCREEN, CustomColor.GREEN, (self.xPos, self.yPos), PlayerParam.RADIUS_OBJECT)
        self.currAngle = 0

        # Is random ?
        self.randomVelo = False

    def _playerInput(self):
        keys = RLParam.ACTIONS
        probs = ObstacleParam.PROBABILITIES_ACTION

        # choosedKey = keys[random.choice(range(len(keys)))]
        randomIndex = random.choices(range(len(keys)), probs)[0]
        choosedKey = keys[randomIndex]

        if choosedKey == PlayerParam.INC_ROTATION_VELO:
            self.currRotationVelocity += ObstacleParam.OBSTACLE_ACCELERATION_ROTATE
        if choosedKey == PlayerParam.DESC_ROTATION_VELO:
            self.currRotationVelocity -= ObstacleParam.OBSTACLE_ACCELERATION_ROTATE
        if choosedKey == PlayerParam.STOP:
            self.currVelocity = 0
            self.currRotationVelocity = 0
        if choosedKey == PlayerParam.INC_FORWARD_VELO:
            self.currVelocity = min(
                self.currVelocity + ObstacleParam.OBSTACLE_ACCELERATION_FORWARD, self.maxVelocity)
            
        if choosedKey == PlayerParam.DESC_FORWARD_VELO:
            self.currVelocity = max(
                self.currVelocity - ObstacleParam.OBSTACLE_ACCELERATION_FORWARD, 0)
        

    def _rayCasting(self):
        pass

    def _checkCollision(self):
        pass

    def draw(self):
        global GLOBAL_SCREEN
        self._playerInput()
        self._move()

        if GameSettingParam.DRAW:
            # draw player on 2D board
            pygame.draw.circle(GLOBAL_SCREEN, CustomColor.GREEN,
                               (self.xPos, self.yPos), PlayerParam.RADIUS_OBJECT)

            pygame.draw.circle(GLOBAL_SCREEN, CustomColor.RED,
                               (self.xPos, self.yPos), 6)
            # draw player direction
            pygame.draw.line(GLOBAL_SCREEN, CustomColor.GREEN, (self.xPos, self.yPos),
                             (self.xPos - math.sin(self.currAngle) * 20,
                              self.yPos + math.cos(self.currAngle) * 20), 3)


class Environment:
    def __init__(self, currentPlayer, currentObstacles):
        self.currPlayer = currentPlayer
        self.currObstacles = currentObstacles

        self.rayCastingData = currentPlayer.rayCastingLists
        self.xPos, self.yPos = currentPlayer.xPos, currentPlayer.yPos

        currentPlayer.mode = MODE_PLAY.RL_TRAIN
        currentPlayer.displayGUI = GUI.DISPLAY

        for obstacle in currentObstacles:
            obstacle.mode = MODE_PLAY.RL_TRAIN
            obstacle.displayGUI = GUI.DISPLAY
            
    def _isDoneEpisode(self):

        dead = False
        if self.yPos <= 0 or self.yPos > GameSettingParam.HEIGHT:
            dead = True
        if self.xPos <= 0 or self.xPos >= GameSettingParam.WIDTH:
            dead = True
        if self.currPlayer.checkCollision():
            dead = True
        return dead

    def _selfUpdated(self):
        # global globalX
        self.rayCastingData = self.currPlayer.rayCastingLists
        self.xPos, self.yPos = self.currPlayer.xPos, self.currPlayer.yPos
        globalX = self.currPlayer.yPos
        # print("(x,y): ", int(self.currPlayer.xPos),int(self.currPlayer.yPos),end="")

    def updateStateByAction(self, actionIndex):
        for obstacle in obstacles:
            obstacle.draw()
            
        self.currPlayer.draw(actionIndex=actionIndex)                    
        self._selfUpdated()
        
        nextState = RLAlgorithm.hashFromDistanceToState(
            signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingData), 
            leftSideDistance=abs(self.xPos), 
            rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
            angle=self.currPlayer.currAngle,
            yver= -1*math.cos(self.currPlayer.currAngle)*self.currPlayer.currVelocity)
        
        reward = RLAlgorithm.getReward(
            currState=nextState, currActionIndex=actionIndex,currPlayer = self.currPlayer)
        
        done = self._isDoneEpisode()
        
        return nextState, reward, done
    
    def getCurrentState(self):
        return RLAlgorithm.hashFromDistanceToState(signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingData),
                                                   leftSideDistance=abs(self.xPos),
                                                   rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
                                                   angle=self.currPlayer.currAngle,
                                                   yver= -1*math.cos(self.currPlayer.currAngle)*self.currPlayer.currVelocity)

    def reset(self):
        # self.currPlayer = None
        # self.currPlayer = Player(maxVelocity=PlayerParam.MAX_VELOCITY,
        #         maxRotationVelocity=PlayerParam.MAX_ROTATION_VELOCITY)
        # self.currObstacles = []
        # for _ in range(ObstacleParam.NUMBER_OF_OBSTACLES):
        #     self.currObstacles.append(Obstacle())
        
        # self.currentPlayer.mode = MODE_PLAY.RL_TRAIN
        # self.currentPlayer.displayGUI = GUI.DISPLAY

        # for obstacle in self.currentObstacles:
        #     obstacle.mode = MODE_PLAY.RL_TRAIN
        #     obstacle.displayGUI = GUI.DISPLAY
            
        # self.currPlayer.draw(actionIndex=2)
        # for obstacle in self.currObstacles:
        #     obstacle.draw()
        del self
        global player, obstacles
        player = Player(maxVelocity=PlayerParam.MAX_VELOCITY,
                maxRotationVelocity=PlayerParam.MAX_ROTATION_VELOCITY) 
        obstacles = []
        for _ in range(ObstacleParam.NUMBER_OF_OBSTACLES):
            obstacles.append(Obstacle())
        return Environment(currentPlayer=player, currentObstacles=obstacles)
                    
###########################################################################################

# Game setting
pygame.init()
GLOBAL_SCREEN = pygame.display.set_mode(
    (GameSettingParam.WIDTH, GameSettingParam.HEIGHT))
pygame.display.set_caption(GameSettingParam.CAPTION)
GLOBAL_CLOCK = pygame.time.Clock()

# Groups
player = Player(maxVelocity=PlayerParam.MAX_VELOCITY,
                maxRotationVelocity=PlayerParam.MAX_ROTATION_VELOCITY)
obstacles = []
for _ in range(ObstacleParam.NUMBER_OF_OBSTACLES):
    obstacles.append(Obstacle())


def startGame(mode=MODE_PLAY.MANUAL):
    if (mode == MODE_PLAY.MANUAL):
        while True:
            GLOBAL_CLOCK.tick(GameSettingParam.FPS)
            GLOBAL_SCREEN.fill(CustomColor.BLACK)
            GLOBAL_SCREEN.blit(GLOBAL_SCREEN, (0, 0))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()

            player.draw(actionIndex=None)            
            for obstacle in obstacles:
                obstacle.draw()

            pygame.display.flip()
    elif (mode == MODE_PLAY.RL_TRAIN):
        # pygame.display.flip()
        # GLOBAL_CLOCK.tick(GameSettingParam.FPS)
        # GLOBAL_SCREEN.fill(CustomColor.BLACK)
        # GLOBAL_SCREEN.blit(GLOBAL_SCREEN, (0, 0))
        
        env = Environment(currentPlayer=player, currentObstacles=obstacles)
        RL = RLAlgorithm(rayCastingData=env.rayCastingData,
                         actions=RLParam.ACTIONS)
        RL.train(env)

# startGame(mode=MODE_PLAY.RL_TRAIN)
startGame(mode=MODE_PLAY.MANUAL)
