import numpy as np
import cv2
import math
import random
import time
import json
import matplotlib.pyplot as plt

listData = []

for i in range(1,435):
    file = open("21-11/table-map-"+str(i)+"0.txt", "r")
    # file = open("curQtable.txt", "r")
    RLInFile = file.read()
    qtb = json.loads(RLInFile)
    trainedParam = 0
    file.close()
    for index in qtb:
        for value in qtb[index]:
            if value != 0:
                trainedParam += 1
    listData.append(trainedParam)
    print(i)

    print(len(qtb))
    break


    
plt.plot(range(1,435),listData)
plt.legend()
plt.show()
