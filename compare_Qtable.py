import numpy as np
import cv2
import math
import random
import time
import json

file = open("19-11/curQtable-1.txt", "r")
RLInFile = file.read()
qtb1 = json.loads(RLInFile)
file.close()

file = open("19-11/curQtable-3.txt", "r")
RLInFile = file.read()
qtb2 = json.loads(RLInFile)
file.close()
print("Load completed")

same = 0
notsame = 0
nocompare = 0

# for index in qtb1:
#     if max(qtb1[index]) == 0 and max(qtb2[index]) == 0:
#         nocompare += 1
#     else:
#         if qtb1[index].index(max(qtb1[index])) != qtb2[index].index(max(qtb2[index])):
#             notsame += 1
#         else:
#             same += 1

print(len(qtb1))

print(same,notsame,nocompare)
