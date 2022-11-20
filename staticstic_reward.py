import re
import matplotlib.pyplot as plt
import numpy as np

file = open("progress.txt", "r+") 
lines = file.readlines()

data = [[],[],[],[],[]]
colorHash = ["red","green","blue"]
labelHash = ["obj","lane","yver"]
print(len(lines))

for line in lines:
    regex = re.findall('-?\d+\.?\d*',line)
    # for i in range(3):
    data[0].append(float(regex[1]))

xCoord = range(len(data[0]))
# xCoord = range(500)
# for i in range(3):
tmp = 0
for j in xCoord:
    tmp = tmp*0.8+data[0][j]*0.2
    data[0][j] = tmp

        
file.close() 

# for i in range(3):
#     plt.plot(xCoord, data[i],label=labelHash[i])
#     meanVal = np.mean(data[i])
#     plt.plot(xCoord, np.full((len(data[0])),meanVal),label="Mean "+labelHash[i])
plt.plot(xCoord, data[0],label="trained param")

plt.legend()
    
plt.show()