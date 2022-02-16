#Main


#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import pandas as pd
import numpy as np
print("start program for importing files")

import os


cwd = os.getcwd() # get current directory of file
filewd = (cwd) # tuple to lock original folder location when changing directory
# print("this is cwd: ", cwd) # check
# print("this is filewd: ", filewd) # check

pointcloudFolder = filewd + "\pointclouds"
#print(pointcloudFolder) #To check

os.chdir(pointcloudFolder)
print("new directory changed to : {0}".format(os.getcwd()))

d = {}
"""the list of point clouds are being put in a dictionary as numpy arrays - not sure if this is the best.
I also considered Pandas. Also wasn't sure whether a list of list would be annoying.
This way we can call them specificaly by name, so that might be the easiest when iterating through them.
"""

for i in range(500):
    # print(i)
    number = "00" + str(i)
    three_digit = number[-3:]
    open_array = np.genfromtxt("{0}.xyz".format(three_digit))
    d["pc{0}".format(three_digit)] = open_array

print(d["pc385"])





#Get feature 1



# seb new lines


#Get feature 2

#Get feature 3

#store feature data

#plot features against each other

#implement clustering