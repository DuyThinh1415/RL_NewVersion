
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

        self.currAngle += self.currRotationVelocity*dt

        self.yPos += math.cos(self.currAngle) * self.currVelocity * dt
        self.xPos += -math.sin(self.currAngle) * self.currVelocity * dt
        
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
                action = 2
                self.currRotationVelocity -= PlayerParam.ACCELERATION_ROTATE
            # Rotate right ()
            if keys[pygame.K_d]:
                action = 1
                self.currRotationVelocity += PlayerParam.ACCELERATION_ROTATE

            # Increase forward velocity
            if keys[pygame.K_w]:
                action = 0
                self.currVelocity = min(
                    self.currVelocity + PlayerParam.ACCELERATION_FORWARD, self.maxVelocity)

            # Stop
            if keys[pygame.K_s]:
                action = 4
                self.currVelocity = 0
                self.currRotationVelocity = 0

            # Decrease forward velocity
            if keys[pygame.K_x]:
                action = 3
                self.currVelocity = max(
                    self.currVelocity - PlayerParam.ACCELERATION_FORWARD, -self.maxVelocity)

            currentState = RLAlgorithm.hashFromDistanceToState(signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingLists),
                                                               leftSideDistance=abs(
                                                                   self.xPos),
                                                               rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
                                                               angle=self.currAngle,
                                                                yver= -1*math.cos(self.currAngle)*self.currVelocity)

            # frame = np.zeros((512,512,3),"uint8")
            # frame = cv2.ellipse(frame,(256,256),(20,20),0,0,360,(0,255,0),-1)
            # for i in range(1,5):
            #     idx = int(currentState[i])*30 + 30
            #     frame = cv2.ellipse(frame,(256,256),(idx,idx),0,180 + i*45-45,180 + i*45,(256,256,128),2)
            # idx = int(currentState[0])*30 + 30
            # frame = cv2.ellipse(frame,(256,256),(idx,idx),0,0,180,(256,256,128),2)

            # idx = int(currentState[5])
            # if idx == 3:
            #     idx = 4
            # elif idx == 4:
            #     idx = 3
            # idx = 4-idx
            # frame = cv2.rectangle(frame,(idx*102,480),(idx*102 + 102,512),(100,255,200),-1)

            # idx = int(currentState[6])
            # if idx == 0:
            #     frame = cv2.ellipse(frame,(256,256),(100,100),0,0,180,(256,0,64),2)
            #     pass
            # else:
            #     frame = cv2.ellipse(frame,(256,256),(100,100),0,180 + idx*60-60,180 + idx*60,(256,0,64),2)
            
            # cv2.putText(frame,currentState[7],(240,260), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,256),1,cv2.LINE_AA)

            # cv2.imshow("robot view",frame)


            # if (action != -1):
            #     print(currentState, " -> ",action)
            # cv2.waitKey(5)

        elif (self.mode == MODE_PLAY.RL_TRAIN):

            if RLParam.ACTIONS[actionIndex] == PlayerParam.INC_ROTATION_VELO:
                self.currRotationVelocity += PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[actionIndex] == PlayerParam.DESC_ROTATION_VELO:
                self.currRotationVelocity -= PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[actionIndex] == PlayerParam.INC_FORWARD_VELO:
                self.currVelocity = min(
                    self.currVelocity + PlayerParam.ACCELERATION_FORWARD, self.maxVelocity)

            if RLParam.ACTIONS[actionIndex] == PlayerParam.DESC_FORWARD_VELO:
                self.currVelocity = max(
                    self.currVelocity - PlayerParam.ACCELERATION_FORWARD, -self.maxVelocity)
        elif (self.mode == MODE_PLAY.DEPLOY):
            currentState = RLAlgorithm.hashFromDistanceToState(signalPerAreaData=RLAlgorithm.convertRayCastingDataToSignalPerArea(rayCastingData=self.rayCastingLists),
                                                               leftSideDistance=abs(
                                                                   self.xPos),
                                                               rightSideDistance=abs(self.xPos - GameSettingParam.WIDTH),
                                                               angle=self.currAngle,
                                                                yver= -1*math.cos(self.currAngle)*self.currVelocity)
            # print(currentState)

            decidedAction = np.argmax(self.deployedQTabled[currentState])

            if RLParam.ACTIONS[decidedAction] == PlayerParam.DESC_ROTATION_VELO:
                self.currRotationVelocity -= PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[decidedAction] == PlayerParam.INC_ROTATION_VELO:
                self.currRotationVelocity += PlayerParam.ACCELERATION_ROTATE

            if RLParam.ACTIONS[decidedAction] == PlayerParam.STOP:
                self.currVelocity = 0
                self.currRotationVelocity = 0

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
            0.6*GameSettingParam.HEIGHT))

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

    def reset(self, newObj = False):
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
    elif (mode == MODE_PLAY.DEPLOY):
        player.mode = MODE_PLAY.DEPLOY
        player.loadQTable()
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


startGame(mode=MODE_PLAY.RL_TRAIN)
# startGame(mode=MODE_PLAY.MANUAL)
# startGame(mode=MODE_PLAY.DEPLOY)
