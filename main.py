
import math
import pygame
from sys import exit
import random
from const import *
from utils import *
from table import *
import datetime
import time
import cv2

# from table import RLAlgorithm

globalX = 0

def getGlobalX():
    return globalX


class Player():
    def __init__(self, maxVelocity, maxRotationVelocity):
        super().__init__()
        global GLOBAL_SCREEN
        self.xPos, self.yPos = PlayerParam.INITIAL_X+random.randint(-150,150), PlayerParam.INITIAL_Y
        self.maxVelocity = maxVelocity
        self.maxRotationVelocity = maxRotationVelocity

        self.currVelocity = 0  # always >= 0
        self.currRotationVelocity = 0  # rotate left < 0, rotate right > 0
        self.currAngle = math.pi + random.randint(-3,3)*math.radians(18)
        self.accelerationForward = PlayerParam.ACCELERATION_FORWARD
        if GameSettingParam.DRAW:
            self.circleRect = pygame.draw.circle(
                GLOBAL_SCREEN, CustomColor.RED, (self.xPos, self.yPos), PlayerParam.RADIUS_OBJECT)

        # Raycasting
        self.rayCastingLists = [PlayerParam.INFINITY] * PlayerParam.CASTED_RAYS

        self.mode = MODE_PLAY.MANUAL

    def _move(self):

        # self.currAngle += self.currRotationVelocity*GameSettingParam.dt

        # self.yPos += math.cos(self.currAngle) * self.currVelocity * GameSettingParam.dt
        # self.xPos += -math.sin(self.currAngle) * self.currVelocity * GameSettingParam.dt

        # self.currRotationVelocity = 0
        # self.currVelocity = 0

        pass
        
        # print(math.degrees(self.currAngle%(math.pi*2)))
    def loadQTable(self):
        if (self.mode == MODE_PLAY.DEPLOY):
            file = open("curQtable.txt", "r+")
            RLInFile = file.read()
            self.deployedQTabled = json.loads(RLInFile)
            file.close()

    def _playerInput(self, actionIndex):
        if (self.mode == MODE_PLAY.MANUAL):
            keys = pygame.key.get_pressed()

            action = -1

            # Rotate left ()
            if keys[pygame.K_a]:
                self.currAngle -= PlayerParam.ACCELERATION_ROTATE
            # Rotate right ()
            if keys[pygame.K_d]:
                self.currAngle += PlayerParam.ACCELERATION_ROTATE


            # Increase forward velocity
            if keys[pygame.K_w]:
                self.yPos += math.cos(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt
                self.xPos += -math.sin(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt

            # Decrease forward velocity
            if keys[pygame.K_s]:
                self.yPos -= math.cos(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt
                self.xPos -= -math.sin(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt

            # Decrease forward velocity
            # if keys[pygame.K_x]:
            #     action = 3
            #     self.currVelocity = max(
            #         self.currVelocity - PlayerParam.ACCELERATION_FORWARD, -self.maxVelocity)

            currentState = RLAlgorithm.hashFromDistanceToState(signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingLists),
                                                               leftSideDistance=abs(
                                                                   self.xPos),
                                                               rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
                                                               angle=self.currAngle,
                                                                yver= -1*math.cos(self.currAngle)*self.currVelocity,
                                                                omega=self.currRotationVelocity)


        elif (self.mode == MODE_PLAY.RL_TRAIN):

            if RLParam.ACTIONS[actionIndex] == PlayerParam.INC_ROTATION_VELO:
                # self.currRotationVelocity = min(self.currRotationVelocity + PlayerParam.ACCELERATION_ROTATE, PlayerParam.MAX_ROTATION_VELOCITY)
                self.currAngle += PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[actionIndex] == PlayerParam.DESC_ROTATION_VELO:
                self.currAngle -= PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[actionIndex] == PlayerParam.INC_FORWARD_VELO:
                self.yPos += math.cos(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt
                self.xPos += -math.sin(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt

            if RLParam.ACTIONS[actionIndex] == PlayerParam.DESC_FORWARD_VELO:
                self.yPos -= math.cos(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt
                self.xPos -= -math.sin(self.currAngle) * PlayerParam.ACCELERATION_FORWARD * GameSettingParam.dt
        elif (self.mode == MODE_PLAY.DEPLOY):
            currentState = RLAlgorithm.hashFromDistanceToState(signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingLists),
                                                               leftSideDistance=abs(
                                                                   self.xPos),
                                                               rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
                                                               angle=self.currAngle,
                                                                yver= -1*math.cos(self.currAngle)*self.currVelocity,
                                                                omega=self.currRotationVelocity)
            # print(currentState)

            decidedAction = np.argmax(self.deployedQTabled[currentState])

            if RLParam.ACTIONS[decidedAction] == PlayerParam.DESC_ROTATION_VELO:
                self.currRotationVelocity -= PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[decidedAction] == PlayerParam.INC_ROTATION_VELO:
                self.currRotationVelocity += PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[decidedAction] == PlayerParam.INC_FORWARD_VELO:
                self.currVelocity = min(
                    self.currVelocity + PlayerParam.ACCELERATION_FORWARD, self.maxVelocity)

            if RLParam.ACTIONS[decidedAction] == PlayerParam.DESC_FORWARD_VELO:
                self.currVelocity = max(
                    self.currVelocity - PlayerParam.ACCELERATION_FORWARD, -self.maxVelocity)

    def _rayCasting(self):
        global obstacles
        inRangedObj = []

        self.rayCastingLists = [PlayerParam.INFINITY] * PlayerParam.CASTED_RAYS

        # stop = False

        for obstacle in obstacles:
            dist = Utils.distanceBetweenTwoPoints(self.xPos, self.yPos, obstacle.xPos, obstacle.yPos)
            if dist < PlayerParam.RADIUS_LIDAR + PlayerParam.RADIUS_OBJECT:
                inRangedObj.append(obstacle)
                # print("alpha: ",alpha)
        startAngle = self.currAngle - PlayerParam.HALF_FOV

        if len(inRangedObj) == 0:
            if GameSettingParam.DRAW:
                for ray in range(PlayerParam.CASTED_RAYS):
                    target_x = self.xPos - math.sin(startAngle) * PlayerParam.RADIUS_LIDAR
                    target_y = self.yPos + math.cos(startAngle) * PlayerParam.RADIUS_LIDAR
                    pygame.draw.line(GLOBAL_SCREEN, CustomColor.GREEN, (self.xPos, self.yPos), (target_x, target_y))
                    startAngle += PlayerParam.STEP_ANGLE 
        else:
            for ray in range(PlayerParam.CASTED_RAYS):
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

                # isDetectObject = PlayerParam.INFINITY
                distance = PlayerParam.INFINITY

                for obstacle in inRangedObj:
                    tvh = (self.xPos-obstacle.xPos)*math.sin(startAngle) + (obstacle.yPos-self.yPos)*math.cos(startAngle)
                    if tvh > 0:
                        distance = min(distance,Utils.getDistanceFromObstacle(self.xPos, self.yPos, target_x, target_y, obstacle.xPos, obstacle.yPos))
                    
                self.rayCastingLists[ray] = distance
                if GameSettingParam.DRAW:
                    if distance <= PlayerParam.RADIUS_LIDAR:
                        target_x = self.xPos - math.sin(startAngle) * distance
                        target_y = self.yPos + math.cos(startAngle) * distance
                        pygame.draw.line(GLOBAL_SCREEN, CustomColor.WHITE, (self.xPos, self.yPos), (target_x, target_y))
                    else:
                        pygame.draw.line(GLOBAL_SCREEN, CustomColor.CRYAN, (self.xPos, self.yPos), (target_x, target_y))
                startAngle += PlayerParam.STEP_ANGLE


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
        # self.checkCollision()
        # nextState = RLAlgorithm.hashFromDistanceToState(
        #     signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingLists), 
        #     leftSideDistance=abs(self.xPos), 
        #     rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
        #     angle=self.currAngle,
        #     yver= -1*math.cos(self.currAngle)*self.currVelocity,
        #     omega=self.currRotationVelocity
        # )
        # print(nextState[-3])
        
        # self._move()
        

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

        self.xPos = random.randint(int(0.1*GameSettingParam.WIDTH), int(
            0.9*GameSettingParam.WIDTH))

        self.yPos = random.randint(0, int( GameSettingParam.HEIGHT-100))

        # self.xPos = 190
        # self.yPos = 300
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
        # self._playerInput()
        # self._move()

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

        for obstacle in currentObstacles:
            obstacle.mode = MODE_PLAY.RL_TRAIN
            
    def _isDoneEpisode(self):

        dead = False
        if self.yPos <= 0 or self.yPos > GameSettingParam.HEIGHT:
            dead = True
        if self.xPos <= 0 or self.xPos >= GameSettingParam.WIDTH:
            dead = True
        if self.currPlayer.checkCollision():
            dead = True
        if (180-abs(math.degrees(self.currPlayer.currAngle)%360 - 180)) < 90:
            dead = True
        return dead

    def _selfUpdated(self):
        # global globalX
        self.rayCastingData = self.currPlayer.rayCastingLists
        self.xPos, self.yPos = self.currPlayer.xPos, self.currPlayer.yPos
        # print("(x,y): ", int(self.currPlayer.xPos),int(self.currPlayer.yPos),end="")

    def updateStateByAction(self, actionIndex):
        # for obstacle in obstacles:
        #     obstacle.draw()
            
        self.currPlayer.draw(actionIndex=actionIndex)    

        self._selfUpdated()

        
        nextState = RLAlgorithm.hashFromDistanceToState(
            signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingData), 
            leftSideDistance=abs(self.xPos), 
            rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
            angle=self.currPlayer.currAngle,
            yver= -1*math.cos(self.currPlayer.currAngle)*self.currPlayer.currVelocity,
            omega=self.currPlayer.currRotationVelocity)

        
        reward = RLAlgorithm.getReward(
            currState=nextState,currPlayer = self.currPlayer, obj = obstacles, action = actionIndex)

        
        done = self._isDoneEpisode()
        
        return nextState, reward, done
    
    def getCurrentState(self):
        return RLAlgorithm.hashFromDistanceToState(signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingData),
                                                   leftSideDistance=abs(self.xPos),
                                                   rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
                                                   angle=self.currPlayer.currAngle,
                                                   yver= -1*math.cos(self.currPlayer.currAngle)*self.currPlayer.currVelocity,
                                                   omega=self.currPlayer.currRotationVelocity)

    def reset(self, newObj = False):
        # self.currPlayer = None
        # self.currPlayer = Player(maxVelocity=PlayerParam.MAX_VELOCITY,
        #         maxRotationVelocity=PlayerParam.MAX_ROTATION_VELOCITY)
        # self.currObstacles = []
        # for _ in range(ObstacleParam.NUMBER_OF_OBSTACLES):
        #     self.currObstacles.append(Obstacle())
        
        # self.currentPlayer.mode = MODE_PLAY.RL_TRAIN

        # for obstacle in self.currentObstacles:
        #     obstacle.mode = MODE_PLAY.RL_TRAIN
            
        # self.currPlayer.draw(actionIndex=2)
        # for obstacle in self.currObstacles:
        #     obstacle.draw()
        curObj = self.currObstacles
        del self
        global player, obstacles
        player = Player(maxVelocity=PlayerParam.MAX_VELOCITY,
                maxRotationVelocity=PlayerParam.MAX_ROTATION_VELOCITY) 

        if newObj:
            obstacles = []
            for _ in range(ObstacleParam.NUMBER_OF_OBSTACLES):
                obstacles.append(Obstacle())
            return Environment(currentPlayer=player, currentObstacles=obstacles)
        else:
            return Environment(currentPlayer=player, currentObstacles=curObj)
                    
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
    player.mode = mode
    if (mode == MODE_PLAY.MANUAL):
        GameSettingParam.DRAW = True
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
        GameSettingParam.DRAW = False
        
        env = Environment(currentPlayer=player, currentObstacles=obstacles)
        RL = RLAlgorithm(rayCastingData=env.rayCastingData,
                         actions=RLParam.ACTIONS)


        
        RL.train(env)
    elif (mode == MODE_PLAY.DEPLOY):
        GameSettingParam.DRAW = True
        player.mode = MODE_PLAY.DEPLOY
        player.loadQTable()
        print("Load Q table Done.")
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

if GameSettingParam.LOCK_QTable:
    print("Warning: Q table is locked !!!")
    cv2.waitKey(1000)


startGame(mode=MODE_PLAY.RL_TRAIN)
# startGame(mode=MODE_PLAY.MANUAL)
# startGame(mode=MODE_PLAY.DEPLOY)
