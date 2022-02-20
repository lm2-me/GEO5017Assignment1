#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import pandas as pd
import numpy as np
import pyoctree as poct
import open3d as o3d
import math as m

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

#current o3d point cloud
def currentPCfile(pointCloudDirectory):
    ## update to all point clouds when testing is done
    pc = "001"
    filewd = os.getcwd()
    folder = "{0}\{1}.xyz".format(filewd,pc)
    print(folder)
    currentPointCloud = o3d.io.read_point_cloud(folder)
    return currentPointCloud

#Get object features for each point cloud height
def allObjectProperties(pointCloudDirectory):
    for pc in pointCloudDirectory:
        #Get current point cloud
        currentPointCloud = currentPC(pointCloudDirectory, pc)

        #Get properties by calling related function
        height = objectHeight(currentPointCloud)
        bBox = boundingBox(currentPointCloud, height)
        avgHeight = objectAverageHeight(currentPointCloud)

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

#Feature 2: Bounding Box (Volume?)
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

#Feature 3: planarity
def planarityPC(pointCloudDirectory):
    pc = currentPCfile(pointCloudDirectory)
    #downsample point cloud with open3d to reduce number of points
    downsampledpc = pc.voxel_down_sample(voxel_size=0.25)
    downsampledpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.5, max_nn=30))
    #o3d.visualization.draw_geometries([downsampledpc], point_show_normal=True)
    print(downsampledpc.normals[0])
    #downsampledpc.estimate_covariances(downsampledpc, search_param=o3d.geometry.KDTreeSearchParamKNN with knn = 30)
    #cos_angle = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / math.sqrt((vec1[0]**2 + vec1[1]**2) * (vec2[0]**2 + vec2[1]**2))


#Get feature 4: Average Height
def objectAverageHeight(currentPointCloud):
    
    npCurrentPointCloud = np.array(currentPointCloud)
    allHeights = npCurrentPointCloud[:,2]

    averageHeight = sum(allHeights) / len(allHeights)
    print(averageHeight)

#Get feature 4: Vertical Slice

#write feature data to file
#
sampleFeatureList = [[0, 50, 4, 5], [1, 10, 5, 2], [2, 15, 4, 3], [3, 20, 4, 5]]

#plot features against each other

#k-means clustering
# value of K
# random K points to act as centroid
# assign data points based on distance
# caluclate new centroid
# re-assign

#Hierachy Clusturing
# Create distance matrix
def hierarchyClustering(npFeatureList):
    #remove 1st index of all lists
    pcnum = npFeatureList[:,0]
    fl = np.delete(npFeatureList, 0, 1)
    #print(fl)
    #compute distance matrix
    distance = lambda p1, p2: m.sqrt(((p1-p2)**2).sum())
    distmatrix = np.asarray([[distance(p1, p2) for p2 in fl] for p1 in fl])
    print(distmatrix)
    
    #get index of closest points
    min = np.min(distmatrix[np.where(distmatrix > 0)])
    min_index = np.array(np.where(distmatrix == min))[0]
    print(min)
    print(min_index)
    level1 = pcnum
    print(level1)
    combinedList = [level1[min_index[0]], level1[min_index[1]]]
    print (combinedList)



#DBSCAN

#Main
if __name__ == "__main__":
    pointCloudDirectory = importFiles()
    #planarityPC(pointCloudDirectory)
    #allObjectProperties(pointCloudDirectory)
    sampleFeatureList = [[0, 50,4,5], [1, 10, 5, 2], [2, 15,4,3], [3, 20,4,5]]
    npFeatureList = np.array(sampleFeatureList)
    hierarchyClustering(npFeatureList)