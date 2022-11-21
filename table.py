from const import *
from utils import *
import random
import numpy as np
import json
import time
import cv2

# from main import getGlobalX

class RLAlgorithm:
    def __init__(self, rayCastingData, actions) -> None:
        file = open("curQtable.txt", "r")
        RLInFile = file.read()
        if not RLInFile:
            print("bruh ???")
        else:
            self.Q = json.loads(RLInFile)
            print("Load completed")

        # self.Q = self._initQTable(actions=actions) 
        self.actions = actions

    def _initQTable(self, actions):
        rs = dict()
        numbersOfLevelRayCasting = 2
        listLevelOfRayCasting = list(range(numbersOfLevelRayCasting))        

        encodedAction = [0] * RLParam.AREA_RAY_CASTING_NUMBERS
        sizeEncodedAction = len(encodedAction)

        for i in range(pow(numbersOfLevelRayCasting, sizeEncodedAction)):
            k = i
            combinedString = ''
            for _ in range(sizeEncodedAction):
                combinedString += str(listLevelOfRayCasting[k %
                                      numbersOfLevelRayCasting])
                k //= numbersOfLevelRayCasting       

            # print(combinedString)     
            
            for level in RLParam.LEVEL_OF_LANE.LIST_LEVEL_OF_LANE:    
                for angle in RLParam.LEVEL_OF_ANGLE.LIST_LEVEL_ANGLES:
                    rs[combinedString + level + angle] = [0] * len(RLParam.ACTIONS)  
                            # print(combinedString + level + angle + yver + omega)

        tmpTable = rs.copy()
        for key in rs:
            for idx in range(len(tmpTable[key])):
                # tmpTable[key][idx] = np.random.randn(1)[0]-0.5
                tmpTable[key][idx] = 0

        return tmpTable

    # @staticmethod
    # def convertRayCastingDataToSignalPerArea(rayCastingData):
    #     pass
    #     numRayEachArea = int(len(rayCastingData)/RLParam.AREA_RAY_CASTING_NUMBERS)
    #     rawSignal = [0]*RLParam.AREA_RAY_CASTING_NUMBERS
    #     signal = [0]*RLParam.AREA_RAY_CASTING_NUMBERS

    #     for i in range(RLParam.AREA_RAY_CASTING_NUMBERS):
    #         rawSignal[i] = min(rayCastingData[numRayEachArea*i:numRayEachArea*(i+1)])

        
    #     for i in range(RLParam.AREA_RAY_CASTING_NUMBERS):
    #         for j in range(len(RLParam.DISTANCE_OF_RAY_CASTING)-1,-1,-1):
    #             if (rawSignal[i] <= RLParam.DISTANCE_OF_RAY_CASTING[j]):
    #                 signal[i] = j
    #     return signal


    @staticmethod
    def hashFromDistanceToState(signalPerAreaData, leftSideDistance, rightSideDistance, angle, yver, omega):  # Tu
        hashFromRayCasting = ""
        for signal in signalPerAreaData:
            hashFromRayCasting += str(signal)

        
        hashFromCenterOfLane = ""
        leftSideDistance = np.clip(int(leftSideDistance*20/GameSettingParam.WIDTH),0,19)
        hashFromCenterOfLane = str(leftSideDistance)
        hashFromAngle = ""
        angle = math.degrees(angle%(math.pi*2))
        
        if angle >= 270:
            hashFromAngle = RLParam.LEVEL_OF_ANGLE.LIST_LEVEL_ANGLES[9]
        elif angle <= 90:
            hashFromAngle = RLParam.LEVEL_OF_ANGLE.LIST_LEVEL_ANGLES[0]
        else:
            hashFromAngle = RLParam.LEVEL_OF_ANGLE.LIST_LEVEL_ANGLES[int((angle-90)/18)]

        # hashFromYver = ""
        # if yver < 0:
        #     hashFromYver = RLParam.Y_VER.BACK
        # else:
        #     yver = min(yver,49)
        #     if (yver < 0):
        #         hashFromYver = RLParam.Y_VER.BACK
        #     elif yver < 20:
        #         hashFromYver = RLParam.Y_VER.FORD1
        #     else:
        #         hashFromYver = RLParam.Y_VER.FORD2

        return hashFromRayCasting + hashFromCenterOfLane + hashFromAngle

    @staticmethod
    def getReward(currState, currPlayer, obj, action):
        
        finalReward = 0
        stateArr = [char for char in currState]

        # Obstacles block car
        minHeight = 10000
        mindis = 10000
        for obstacle in obj:
            theda = math.sqrt((obstacle.xPos - currPlayer.xPos)**2+(obstacle.yPos - currPlayer.yPos)**2)
            beta = 0
            if obstacle.xPos - currPlayer.xPos < 0:
                beta = math.acos((obstacle.yPos - currPlayer.yPos)/theda) - currPlayer.currAngle
            else:
                beta = math.acos((obstacle.yPos - currPlayer.yPos)/theda) + currPlayer.currAngle
            height = abs(theda*math.sin(beta))
            tvh = (currPlayer.xPos-obstacle.xPos)*math.sin(currPlayer.currAngle) + (obstacle.yPos-currPlayer.yPos)*math.cos(currPlayer.currAngle)
            distance = Utils.distanceBetweenTwoPoints(
                currPlayer.xPos, currPlayer.yPos, obstacle.xPos, obstacle.yPos)
            if distance < PlayerParam.RADIUS_LIDAR*0.5 and tvh > 0:
                mindis = min(distance,mindis)
                minHeight = min(minHeight,height)

        if minHeight < 30:
            if SOME_PARAM_FOR_CODE_DO.preMinHeight == -1:
                SOME_PARAM_FOR_CODE_DO.preMinHeight = minHeight 
            else:
                finalReward += (minHeight - SOME_PARAM_FOR_CODE_DO.preMinHeight)*15
                SOME_PARAM_FOR_CODE_DO.preMinHeight = minHeight 

        if minHeight < 10 and RLParam.ACTIONS[action] == PlayerParam.INC_FORWARD_VELO:
            finalReward -= 50

        # finalReward += (PlayerParam.RADIUS_LIDAR*0.6 - mindis)*0.2


        #must go if nothing ahead

        if currPlayer.xPos < GameSettingParam.WIDTH/2 and RLParam.ACTIONS[action] == PlayerParam.DESC_ROTATION_VELO:            
            finalReward -= 50
        if currPlayer.xPos > GameSettingParam.WIDTH/2 and RLParam.ACTIONS[action] == PlayerParam.INC_ROTATION_VELO:
            finalReward -= 50

        if RLParam.ACTIONS[action] == PlayerParam.DO_NOTHING:
            finalReward -= 200
        if RLParam.ACTIONS[action] == PlayerParam.DESC_FORWARD_VELO:
            finalReward -= 100

        if RLParam.ACTIONS[action] == PlayerParam.INC_ROTATION_VELO or RLParam.ACTIONS[action] == PlayerParam.DESC_ROTATION_VELO:
            finalReward -= 50

        # Car out of lane

        if currPlayer.xPos > GameSettingParam.WIDTH/2:
            finalReward -= (currPlayer.xPos - SOME_PARAM_FOR_CODE_DO.preX)*10
        else:
            finalReward +=( currPlayer.xPos - SOME_PARAM_FOR_CODE_DO.preX)*10
        finalReward -= 10

        SOME_PARAM_FOR_CODE_DO.preX = currPlayer.xPos

            # SOME_PARAM_FOR_CODE_DO.preX

        # Prevent stop and go back action
        finalReward += 2*(SOME_PARAM_FOR_CODE_DO.preY - currPlayer.yPos)

        SOME_PARAM_FOR_CODE_DO.preY = currPlayer.yPos


        if (180-abs(math.degrees(currPlayer.currAngle)%360 - 180)) > 90:
            finalReward +=( (180-abs(math.degrees(currPlayer.currAngle)%360 - 180))/5-18)*3
        else:
            finalReward += RLParam.GO_FUCKING_DEAD

        if currPlayer.checkCollision():
            finalReward += RLParam.GO_FUCKING_DEAD

        if currPlayer.xPos < 0 or  currPlayer.xPos > GameSettingParam.WIDTH:
            finalReward += RLParam.GO_FUCKING_DEAD

        if currPlayer.yPos > GameSettingParam.HEIGHT:
            finalReward += RLParam.GO_FUCKING_DEAD
            
        return finalReward

    def _epsilonGreedyPolicy(self, currState):
        if np.random.uniform(0, 1) < RLParam.EPSILON:
            return random.choice(range(len(self.actions)))
        else:
            return np.argmax(self.Q[currState])

    def train(self, env):
        alphas = np.linspace(
            RLParam.MAX_ALPHA, RLParam.MIN_ALPHA, RLParam.N_EPISODES)
            
        # hashState = "><=^~"
        # file = open("track_reward.txt", "w")
        # file.write("")
        # file.close()

        visualMap = np.zeros((GameSettingParam.HEIGHT,GameSettingParam.WIDTH,3),dtype="uint8")
        progressFile = open("progress.txt", "w")
        outputVideo = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (GameSettingParam.WIDTH,GameSettingParam.HEIGHT))
        startTime = time.time()
        e = -1
        mapCounter  = 0
        numPassOnThisMap = 0
        numTryOnThisMap = 0
        passIn = 0
        mapCounter = 0
        prePass = 0
        while True:
            
            e+=1
            state = env.getCurrentState()
            totalReward = 0
            alpha = 1
            if (e < RLParam.N_EPISODES):
                alpha = alphas[e]

            # if (e%10000 == 0):
            #     RLParam.EPSILON = max(RLParam.EPSILON - 0.05,0.05)
            # startTime = time.time()
            # stateList = ""
            startPoint = (int(env.xPos),int(env.yPos))

            unTrainedParam = 0
            SOME_PARAM_FOR_CODE_DO.preMinHeight = -1
            SOME_PARAM_FOR_CODE_DO.preX = env.xPos
            SOME_PARAM_FOR_CODE_DO.preY = env.yPos

            
            done = False
            curEpochMap = np.zeros((GameSettingParam.HEIGHT,GameSettingParam.WIDTH,3),dtype="uint8")
            

            for obj in env.currObstacles:
                curEpochMap = cv2.circle(curEpochMap,(int(obj.xPos),int(obj.yPos)),20,(0,0,255),-1)

            for actionCount in range(RLParam.MAX_EPISODE_STEPS):
                
                # print(" action:",actionCount,"episode:",e+1,"state:", state, "currentReward",totalReward,end="   ")
                
                actionIndex = self._epsilonGreedyPolicy(currState=state)
                nextState, reward, done = env.updateStateByAction(actionIndex)
                # if random.random() < 0.001 and state != nextState:
                #     print(" -------------- Epoch",e," State ",actionCount," ---------------")
                #     print("currentState:",state," :nextState:",nextState)
                #     for i in range(len(RLParam.ACTIONS)):
                #         print(i," ",RLParam.ACTIONS[i],":",self.Q[state][i]," | ",self.Q[nextState][i])
                #     print("action:",actionIndex," reward:",reward)
                #     print()
                #     cv2.waitKey(0)
                totalReward += reward
                if self.Q[state][actionIndex] == 0:
                    unTrainedParam += 1
                self.Q[state][actionIndex] = (1-alpha)*self.Q[state][actionIndex] + \
                    alpha * (reward + RLParam.GAMMA *
                             np.max(self.Q[nextState]))
                state = nextState

                if e%20 == 0 and actionCount%10 == 0:
                    curPoint = (int(env.xPos),int(env.yPos))
                    drawColor = (255,0,255)
                    if (int(actionCount/100)%2 == 0):
                        drawColor = (0,255,0)

                    curEpochMap = cv2.line(curEpochMap,curPoint,startPoint,drawColor,2)
                    # cv2.imshow("current Path",curEpochMap)
                    startPoint = curPoint

                
                # stateList += hashState[actionIndex]
                # if actionCount%100 == 0 and actionCount != 0:
                #     stateList +="\n"
                # print("state:", state)

                if done: 
                    # totalReward -=  (actionCount + 1) * 0.01 # 120s * 1 = 120
                    break
            if e%20 == 0:
                cv2.addWeighted(visualMap,0.5,curEpochMap,1,0.0,visualMap)
                visualMapWText = visualMap.copy()
                visualMapWText = cv2.putText(visualMapWText,str(int(totalReward)),(20,GameSettingParam.HEIGHT - 50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(e),(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(actionCount),(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(passIn),(20,60),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(numPassOnThisMap),(300,40),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                cv2.imshow("Last Path",visualMapWText)
                outputVideo.write(visualMapWText)
            # print("step time: ",time.time() - startTime)
            # comment = f"{e + 1}: act {actionCount} -> {totalReward}, map {mapCounter} -=- "
            # if e%10 == 0 and e != 0:
            #     comment += "\n"
            # comment += stateList+"\n"

            # progressFile.write(comment)
            # if env.yPos < 0:
                # print(f"Episode {e} trained {(time.time() - startTime)/(e+1)}s : rw in {actionCount} actions: {totalReward}, map {mapCounter} try {numTryOnThisMap} times")

            if (e%5000 == 0 and e!=0 and GameSettingParam.LOCK_QTable == False):
                print(e," Epoch done.")
                file = open("curQtable.txt", "w")
                file.write(json.dumps(self.Q))
                file.close()

            #     print("Done")
            RLParam.EPSILON -= 0.0001
            if RLParam.EPSILON < 0.001:
                RLParam.EPSILON = 0.001
            numTryOnThisMap += 1
            if env.yPos < 0:
                numPassOnThisMap += 1
                
                comment = f"map {mapCounter} pass {numPassOnThisMap} times on {numTryOnThisMap - prePass} trys, avg time is {(time.time() - startTime)/(e+1)}"
                print(comment)
                prePass = numTryOnThisMap
                progressFile.write(comment)
                
                # passIn = actionCount
            if numPassOnThisMap >= 100:
                comment = f"map {mapCounter} pass in {numTryOnThisMap} trys, avg time is {(time.time() - startTime)/(e+1)} ---------------------- "
                print(comment)
                RLParam.EPSILON = min(0.01,0.5 - mapCounter*0.01)
                progressFile.write(comment)
                numPassOnThisMap = 0
                numTryOnThisMap = 0
                prePass = 0
                mapCounter += 1
                file = open("21-11/table-map-"+str(mapCounter)+".txt", "w")
                file.write(json.dumps(self.Q))
                file.close()
                env = env.reset(True)
            else:
                env = env.reset(False)
            # env = env.reset(False)

            if cv2.waitKey(1) == 27:
                break

            
        print("Save Q table")

        outputVideo.release()
        if GameSettingParam.LOCK_QTable == False:
            file = open("curQtable.txt", "w")
            file.write(json.dumps(self.Q))
            file.close()

        progressFile.close()

        print(" ====== Train Completed ==========")
