import re
import matplotlib.pyplot as plt
import numpy as np

file = open("progress.txt", "r+") 
lines = file.readlines()

def getEpisode(regex):
    return int(regex[0])

def getX(regex):
    return float(regex[1])

def getY(regex):
    return float(regex[2])

def getTotalActions(regex):
    return int(regex[3])

def getTotalReward(regex):
    return float(regex[4])


episodeList = []
xPosList = []
yPosList = []
totalActionsList = []
totalRewardList = []

def genMean(list, step):
    newList = []
    for i in range(int(len(list)/step)-1):
        newList.append(np.mean(list[i*step:(i+1)*step]))
    return newList

for line in lines:
    regex = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    if (len(regex) == 0):
        continue
    
    episode = getEpisode(regex)
    xPos = getX(regex)
    yPos = getY(regex)
    totalAction = getTotalActions(regex)
    totalReward = getTotalReward(regex)
    
    episodeList.append(episode)
    xPosList.append(xPos)
    yPosList.append(yPos)
    totalActionsList.append(totalAction)
    totalRewardList.append(totalReward)

step = 20

xPosList = genMean(xPosList,step)
yPosList = genMean(yPosList,step)
totalActionsList = genMean(totalActionsList,step)
totalRewardList = genMean(totalRewardList,step)
episodeList = range(len(totalActionsList))

# print(xPosList)
        
file.close() 
# print(totalRewardList)
fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(episodeList, xPosList)
axs[0, 0].set_title('xPos per episode')
axs[0, 1].plot(episodeList, yPosList, 'tab:orange')
axs[0, 1].set_title('yPos per episode')
axs[1, 0].plot(episodeList, totalActionsList, 'tab:green')
axs[1, 0].set_title('Total actions per episode')
axs[1, 1].plot(episodeList, totalRewardList, 'tab:red')
axs[1, 1].set_title('Total rewards per episode')

fig.tight_layout()
    
plt.show()