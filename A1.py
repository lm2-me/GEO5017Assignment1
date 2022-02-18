#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import pandas as pd
import numpy as np
import pyoctree as poct
import open3d as o3d
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

#visualize point cloud
def visualizePC(pointCloudDirectory):
    pc = currentPCfile(pointCloudDirectory)
    o3d.visualization.draw_geometries([pc])

#visualize point cloud
def currentPCfile(pointCloudDirectory):
    pc = "001"
    filewd = os.getcwd()
    folder = "{0}\{1}.xyz".format(filewd,pc)
    print(folder)
    currentPointCloud = o3d.io.read_point_cloud(folder)
    return currentPointCloud

#check planarity
def planarityPC(pointCloudDirectory):
    pc = currentPCfile(pointCloudDirectory)
    #downsample point cloud with open3d to reduce number of points
    downsampledpc = pc.voxel_down_sample(voxel_size=0.25)
    downsampledpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.5, max_nn=30))
    o3d.visualization.draw_geometries([downsampledpc], point_show_normal=True)


#Get object features for each point cloud height
def allObjectProperties(pointCloudDirectory):
    for pc in pointCloudDirectory:
        #Get current point cloud
        currentPointCloud = currentPC(pointCloudDirectory, pc)

        #Get properties by calling related function
        height = objectHeight(currentPointCloud)
        bBox = boundingBox(currentPointCloud, height)
        avgHeight = objectAverageHeight(currentPointCloud)
        numPoints = len(currentPointCloud)

        #print("height: " + str(height) + " bounding box: " + str(bBox) + "number of points: " + str(numPoints))

#Get current point cloud
def currentPC(pointCloudDirectory, pc):
    number = "00" + str(pc)
    three_digit = number[-3:]
    name = "pc" + three_digit
    currentPointCloud = pointCloudDirectory[name]
    return currentPointCloud

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


#Get feature 3: planarity

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
    planarityPC(pointCloudDirectory)
    #allObjectProperties(pointCloudDirectory)