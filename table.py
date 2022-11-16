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
        self.signalPerAreaData = self.convertRayCastingDataToSignalPerArea(
            rayCastingData=rayCastingData)
        file = open("curQtable.txt", "r")
        RLInFile = file.read()
        if not RLInFile:
            # self.Q = self._initQTable(actions=actions) 
            print("bruh ???")
        else:
            self.Q = json.loads(RLInFile)
            print("Load completed")

        # self.Q = self._initQTable(actions=actions) 
        self.actions = actions

    def _initQTable(self, actions):
        # https://www.geeksforgeeks.org/print-all-the-permutation-of-length-l-using-the-elements-of-an-array-iterative/
        rs = dict()
        numbersOfLevelRayCasting = len(RLParam.DISTANCE_OF_RAY_CASTING)
        listLevelOfRayCasting = list(range(numbersOfLevelRayCasting))        

        encodedAction = [0] * (len(self.signalPerAreaData))
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
                    for yver in RLParam.Y_VER.LIST_LEVEL_YVER:

                        rs[combinedString + level + angle + yver] = [0] * len(RLParam.ACTIONS)  





        tmpTable = rs.copy()
        for key in rs:
            for idx in range(len(tmpTable[key])):
                # tmpTable[key][idx] = np.random.randn(1)[0]-0.5
                tmpTable[key][idx] = 0

        return tmpTable

    @staticmethod
    def convertRayCastingDataToSignalPerArea(rayCastingData):
        # print("rayCastingData",rayCastingData)
        div = len(rayCastingData) // RLParam.AREA_RAY_CASTING_NUMBERS
        mod = len(rayCastingData) % RLParam.AREA_RAY_CASTING_NUMBERS

        tmpData = [0] * (RLParam.AREA_RAY_CASTING_NUMBERS +
                         (0 if mod == 0 else 1))
        
        tmpList = [22,33,44,55,66,88]
        # for i in range(len(tmpData)):
        #     tmp = []
        #     for _ in range(div):
        #         tmp.append(rayCastingData[tmpCount])
        #         tmpCount += 1
        #         if (tmpCount == len(rayCastingData)):
        #             break
        #     tmpData[i] = min(tmp)
        # return tmpData
        tmptmptmp = [[],[],[],[],[],[]]
        for i in range(len(rayCastingData)):
            for j in range(len(tmpList)):
                if i < tmpList[j]:
                    tmptmptmp[j].append(rayCastingData[i])
                    break
        tmptmptmp[0].append(min(tmptmptmp[5]))
        for i in range(5):
            tmpData[i] = min(tmptmptmp[i])
        return tmpData[0:RLParam.AREA_RAY_CASTING_NUMBERS]



    @staticmethod
    def hashFromDistanceToState(signalPerAreaData, leftSideDistance, rightSideDistance, angle, yver):  # Tu
        hashFromRayCasting = ""
        for signal in signalPerAreaData:
            for index, distanceRange in enumerate(RLParam.DISTANCE_OF_RAY_CASTING):
                if index == len(RLParam.DISTANCE_OF_RAY_CASTING) - 1:
                    hashFromRayCasting += RLParam.LEVEL_OF_RAY_CASTING.INFINITY
                    break
                elif signal < distanceRange:
                    hashFromRayCasting += str(index)
                    break
        hashFromCenterOfLane = ""
        distanceFromCenterOfLane = abs(
            leftSideDistance - rightSideDistance) / 2
        for index, distance in enumerate(RLParam.DISTANCE_FROM_CENTER_OF_LANE):
            if index == len(RLParam.DISTANCE_FROM_CENTER_OF_LANE) - 1:
                hashFromCenterOfLane += RLParam.LEVEL_OF_LANE.MIDDLE
                break
            elif distanceFromCenterOfLane > distance:
                if leftSideDistance < rightSideDistance:
                    hashFromCenterOfLane += str(index +
                                                 int(RLParam.LEVEL_OF_LANE.MIDDLE) + 1)
                else:
                    hashFromCenterOfLane += str(index)
                break
        hashFromAngle = ""
        angle = math.degrees(angle%(math.pi*2))
        
        if angle > 270 or angle < 90:
            hashFromAngle = RLParam.LEVEL_OF_ANGLE.BACK
        else:
            hashFromAngle = RLParam.LEVEL_OF_ANGLE.LIST_LEVEL_ANGLES[int((angle-90)/60)+1]
        # print(angle ," => ",hashFromAngle)

        hashFromYver = ""
        if yver < 0:
            hashFromYver = RLParam.Y_VER.BACK
        else:
            yver = min(yver,49)
            if (yver < 0):
                hashFromYver = RLParam.Y_VER.BACK
            elif yver < 20:
                hashFromYver = RLParam.Y_VER.FORD1
            else:
                hashFromYver = RLParam.Y_VER.FORD2

        # print(hashFromCenterOfLane)

        return hashFromRayCasting + hashFromCenterOfLane + hashFromAngle + hashFromYver

    @staticmethod
    def getReward(currState, currActionIndex, currPlayer):
        # progressFile = open("track_reward.txt", "a")
        
        # comment = "obj:"
        finalReward = 0
        # lastReward = 0
        stateArr = [char for char in currState]
        lidarStates = stateArr[0:RLParam.AREA_RAY_CASTING_NUMBERS]
        centerState = stateArr[RLParam.AREA_RAY_CASTING_NUMBERS]

        # Obstacles block car
        lidarState = min(lidarStates)
        if lidarState == 0:
            finalReward += -200
        elif lidarState == 1:
            finalReward += -20
        else:
            finalReward += 5

        # comment += str(finalReward-lastReward)+" lane:"
        # lastReward = finalReward

        # Car out of lane
        if centerState == RLParam.LEVEL_OF_LANE.MIDDLE:
            finalReward += 10
        elif centerState == RLParam.LEVEL_OF_LANE.RIGHT or centerState == RLParam.LEVEL_OF_LANE.LEFT:
            finalReward += -5
        elif centerState == RLParam.LEVEL_OF_LANE.MOST_RIGHT or centerState == RLParam.LEVEL_OF_LANE.MOST_LEFT:
            finalReward += -100

        # finalReward += -0.05*abs(currPlayer.xPos - GameSettingParam.WIDTH/2)

        # comment += str(finalReward-lastReward)+" yVer:"
        # lastReward = finalReward

        # Prevent stop and go back action
        y_Ver = math.cos(currPlayer.currAngle)*currPlayer.currVelocity
        finalReward += -0.5*y_Ver

        # comment += str(finalReward-lastReward)+" collision:"
        # lastReward = finalReward

        if currPlayer.checkCollision():
            finalReward += RLParam.GO_FUCKING_DEAD

        # comment += str(finalReward-lastReward)+" outOfRoad:"
        # lastReward = finalReward


        if currPlayer.xPos < 0 or  currPlayer.xPos > GameSettingParam.WIDTH:
            finalReward += RLParam.GO_FUCKING_DEAD

        if currPlayer.yPos < 0:
            finalReward += 100000 # heel yeah

        if currPlayer.yPos > GameSettingParam.HEIGHT:
            finalReward += RLParam.GO_FUCKING_DEAD

        # comment += str(finalReward-lastReward) + "\n"
        # progressFile.write(comment)
            
        return finalReward

    def _epsilonGreedyPolicy(self, currState):
        if np.random.uniform(0, 1) < RLParam.EPSILON:
            return random.choice(range(len(self.actions)))
        else:
            return np.argmax(self.Q[currState])

    def train(self, env):
        alphas = np.linspace(
            RLParam.MAX_ALPHA, RLParam.MIN_ALPHA, RLParam.N_EPISODES)

        maxReward = RLParam.GO_FUCKING_DEAD
        maxQ = self.Q
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
        while True:
            
            e+=1
            state = env.getCurrentState()
            totalReward = 0
            alpha = 1
            if (e < RLParam.N_EPISODES):
                alpha = alphas[e]

            if (e%10000 == 0):
                RLParam.EPSILON = max(RLParam.EPSILON - 0.05,0.05)
            # startTime = time.time()
            # stateList = ""
            startPoint = (PlayerParam.INITIAL_X,PlayerParam.INITIAL_Y)

            unTrainedParam = 0
            
            done = False
            curEpochMap = np.zeros((GameSettingParam.HEIGHT,GameSettingParam.WIDTH,3),dtype="uint8")

            for obj in env.currObstacles:
                curEpochMap = cv2.circle(curEpochMap,(int(obj.xPos),int(obj.yPos)),20,(0,0,255),-1)

            for actionCount in range(RLParam.MAX_EPISODE_STEPS):
                
                # print(" action:",actionCount,"episode:",e+1,"state:", state, "currentReward",totalReward,end="   ")
                
                actionIndex = self._epsilonGreedyPolicy(currState=state)
                nextState, reward, done = env.updateStateByAction(actionIndex)
                totalReward += reward
                if self.Q[state][actionIndex] == 0:
                    unTrainedParam += 1
                self.Q[state][actionIndex] = (1-alpha)*self.Q[state][actionIndex] + \
                    alpha * (reward + RLParam.GAMMA *
                             np.max(self.Q[nextState]))
                state = nextState

                if actionCount%20 == 0:
                    curPoint = (int(env.xPos),int(env.yPos))
                    drawColor = (255,0,255)
                    if (int(actionCount/100)%2 == 0):
                        drawColor = (0,255,0)

                    curEpochMap = cv2.line(curEpochMap,curPoint,startPoint,drawColor,2)
                    startPoint = curPoint

                
                # stateList += hashState[actionIndex]
                # if actionCount%100 == 0 and actionCount != 0:
                #     stateList +="\n"
                # print("state:", state)


                

                if done or actionCount == RLParam.MAX_EPISODE_STEPS - 1: 
                    # totalReward -=  (actionCount + 1) * 0.01 # 120s * 1 = 120
                    break
            if e%1 == 0:
                cv2.addWeighted(visualMap,0.5,curEpochMap,1,0.0,visualMap)
                visualMapWText = visualMap.copy()
                visualMapWText = cv2.putText(visualMapWText,str(int(totalReward)),(20,GameSettingParam.HEIGHT - 50),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(e),(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(actionCount),(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                visualMapWText = cv2.putText(visualMapWText,str(numPassOnThisMap),(300,40),cv2.FONT_HERSHEY_SIMPLEX,0.8,(199,141,255),1,cv2.LINE_AA)
                cv2.imshow("Last Path",visualMapWText)
                outputVideo.write(visualMapWText)
            # print("step time: ",time.time() - startTime)
            comment = f"{e + 1}: act {actionCount} -> {totalReward}, map {mapCounter} -=- "
            if e%10 == 0 and e != 0:
                comment += "\n"
            # comment += stateList+"\n"


            if totalReward > maxReward:
                maxReward = totalReward
                maxQ = self.Q

            # RLParam.EPSILON = max(RLParam.EPSILON-0.0002,0)

            progressFile.write(comment)
            
            print(f"Episode {e} trained {(time.time() - startTime)/(e+1)}s : rw {actionCount} actions: {totalReward}, max reward: {maxReward}")

            if (e%5000 == 0 and e!=0):
                print("Save mode -------------- ",end="")
                file = open("D:/RL/curQtable.txt", "w")
                file.write(json.dumps(self.Q))
                file.close()

                file = open("D:/RL/maxReward.txt", "w")
                file.write(json.dumps(maxQ))
                file.close()

                print("Done")

            if env.yPos < 0:
                numPassOnThisMap += 1
            if numPassOnThisMap > 5:
                numPassOnThisMap = 0
                env = env.reset(True)
            else:
                env = env.reset(False)


            

            if cv2.waitKey(1) == 27:
                break

            
        print("Save Q table")

        outputVideo.release()
        file = open("D:/RL/curQtable.txt", "w")
        file.write(json.dumps(self.Q))
        file.close()

        file = open("D:/RL/maxReward.txt", "w")
        file.write(json.dumps(maxQ))
        file.close()

        progressFile.close()

        print(" ====== Train Completed ==========")
