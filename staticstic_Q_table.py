import numpy as np
import cv2
import math
import random
import time
import json

file = open("curQtable.txt", "r")
RLInFile = file.read()
qtb = json.loads(RLInFile)
print("Load completed")

# 0000 00000000

paramStaticstic = []

for i in range(5):
    paramStaticstic.append([0,0])

# train in idx0, untrain in idx1

numbersOfLevelRayCasting = 3
listLevelOfRayCasting = list(range(numbersOfLevelRayCasting))        

encodedAction = [0] * (5)
sizeEncodedAction = len(encodedAction)

# for i in range(pow(numbersOfLevelRayCasting, sizeEncodedAction)):
#     k = i
#     combinedString = ''
#     for _ in range(sizeEncodedAction):
#         combinedString += str(listLevelOfRayCasting[k %
#                                 numbersOfLevelRayCasting])
#         k //= numbersOfLevelRayCasting       

#     # print(combinedString)     
    
#     for level in ["0","1","2","3","4"]:    
#         for angle in ["0","1","2","3"]:
#             for yver in ["0","1","2"]:
#                 # for eachValue in qtb[combinedString+level+angle+yver]:
#                     # counter =
#                 if qtb[combinedString+level+angle+yver] == [0]*5:
#                     paramStaticstic[0][1] += 1
#                 else:
#                     paramStaticstic[0][0] += 1

# for index in qtb:
#     for idx in len(qtb[index]):
#         if qtb[index][idx] == 0:
#             for i in range(8):
#                 paramStaticstic[i][1] += 1
#         else:
#             for i in range(8):
#                 paramStaticstic[i][0] += 1

for colCehck in range(9):

    paramStaticstic = []

    for i in range(5):
        paramStaticstic.append([0,0])

    for index in qtb:
        for value in qtb[index]:
            if value == 0:
                paramStaticstic[int(index[colCehck])][1] += 1
            else:
                paramStaticstic[int(index[colCehck])][0] += 1

    # print(qtb["000000000400"])
    print("paramStaticstic:",colCehck)
    for i in range(5):
        print("idx ",i," trained:",paramStaticstic[i][0]," | untrained:",paramStaticstic[i][1])