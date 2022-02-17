#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import pandas as pd
import numpy as np
print("start program for importing files")

import os

def importFiles():
    cwd = os.getcwd() # get current directory of file
    #print("cwd = " + cwd)
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
    
    #print(d["pc385"])
    return d


#Get object features for each point cloud height
def allObjectProperties(pointCloudDirectory):
    for pc in pointCloudDirectory:
        #Get current point cloud
        number = "00" + str(pc)
        three_digit = number[-3:]
        name = "pc" + three_digit
        currentPointCloud = pointCloudDirectory[name]

        #Get properties by calling related function
        height = objectHeight(currentPointCloud)
        bBox = boundingBox(currentPointCloud, height)
        avgHeight = objectAverageHeight(currentPointCloud)
        numPoints = len(currentPointCloud)

        #print("height: " + str(height) + " bounding box: " + str(bBox) + "number of points: " + str(numPoints))

#Get feature 1: Height
def objectHeight(currentPointCloud):
    maxZ = 0

    for point in currentPointCloud:
        if point[2] > maxZ:
            maxZ = point[2]
        #maxZ now largest Z

    height = maxZ
    #print(height)
    return height

#Create Bounding Box
def boundingBox(currentPointCloud, height):
    maxX = 0
    minX = 100
    maxY = 0
    minY = 100

    for point in currentPointCloud:
        if point[0] < minX:
            minX = point[0]
        if point[0] > maxX:
            maxX = point[0]
        if point[1] < minY:
            minY = point[1]
        if point[1] > maxY:
            maxY = point[1]
        #minX, minY, maxX, maxY now defined
    
    minCorner = (minX, minY, 0)
    maxCorner = (maxX, maxY, height)

    bBox = (minCorner, maxCorner)
    
    #print(bBox)
    return bBox

#Get feature 2: Vertical Slices


#Get feature 3:

#Get feature 4: Average Height
def objectAverageHeight(currentPointCloud):
    
    npCurrentPointCloud = np.array(currentPointCloud)
    allHeights = npCurrentPointCloud[:,2]

    averageHeight = sum(allHeights) / len(allHeights)
    print(averageHeight)

#store feature data

#plot features against each other

#implement clustering

#Main
if __name__ == "__main__":
    pointCloudDirectory = importFiles()
    allObjectProperties(pointCloudDirectory)