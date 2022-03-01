#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import pandas as pd
import numpy as np
import pyoctree as poct
import open3d as o3d
import math as m
import copy

import Hierarchy as hc
import ClusterComparing as cc
import KMeans
import DBSCAN
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
def visualizePC(pointCloudDirectory, pc):
    cloud = currento3dPCfile(pointCloudDirectory, pc)
    o3d.visualization.draw_geometries([cloud])

#current o3d point cloud
def currento3dPCfile(pc):
    ## update to all point clouds when testing is done
    
    number = "00" + str(pc)
    three_digit = number[-3:] 
    filewd = os.getcwd()
    folder = "{0}\{1}.xyz".format(filewd, three_digit)
    #print(folder)
    currentPointCloud = o3d.io.read_point_cloud(folder)
    return currentPointCloud

#Get object features for each point cloud height
def allObjectProperties(pointCloudDirectory):
    i = 0
    object_features = []
    for pc in pointCloudDirectory:
        #Get current point cloud
        currentPointCloud = currentPC(pointCloudDirectory, pc)
        currentPointCloud_o3d = currento3dPCfile(pc)

        #Get properties by calling related function
        height = objectHeight(currentPointCloud)
        volume = convexHull(currentPointCloud_o3d)
        #avg_height = objectAverageHeight(currentPointCloud)
        area = areaBase(currentPointCloud_o3d)
        num_planes = planarityPC(currentPointCloud_o3d)

        if i >=500: break

        object_features.append([i, height, volume, area])
        i += 1
        #print("height: " + str(height) + " volume: " + str(volume) + " area: " + str(area))
    return object_features

#Get current point cloud and save to new array
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

#Feature 2: Convex hull for Volume
def convexHull(pc):
    convhull, _ = pc.compute_convex_hull()
    #print('convhull', convhull)
    convhull_lns = o3d.geometry.LineSet.create_from_triangle_mesh(convhull)
    convhull_lns.paint_uniform_color((0, 0, 1))
    #visualize convex hull
    #o3d.visualization.draw_geometries([pc, convhull_lns])
    
    volume = convhull.get_volume()
    #print(volume)
    return volume

#Feature 3: planarity
def planarityPC(pc):
    #downsample point cloud with open3d to reduce number of points
    downsampledpc = pc.voxel_down_sample(voxel_size=0.25)
    downsampledpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.5, max_nn=30))
    #o3d.visualization.draw_geometries([downsampledpc], point_show_normal=True)
    #print(downsampledpc.normals[0])
    #downsampledpc.estimate_covariances(downsampledpc, search_param=o3d.geometry.KDTreeSearchParamKNN with knn = 30)
    #cos_angle = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / math.sqrt((vec1[0]**2 + vec1[1]**2) * (vec2[0]**2 + vec2[1]**2))


#Get feature 4: Average Height
def objectAverageHeight(currentPointCloud):
    
    npCurrentPointCloud = np.array(currentPointCloud)
    allHeights = npCurrentPointCloud[:,2]

    averageHeight = sum(allHeights) / len(allHeights)
    #print(averageHeight)
    return averageHeight

#Get feature 5: Area of plan view
def areaBase(pc):
    bBox = pc.get_axis_aligned_bounding_box()
    bBox.color = (0, 0, 1)

    min = bBox.get_min_bound()
    max = bBox.get_max_bound()

    length = max[0] - min[0]
    width = max[1] - min[1]

    area = length * width   
    
    bBox.color = (0, 0, 1)
    #o3d.visualization.draw_geometries([pc, bBox])
    return area

#Main
if __name__ == "__main__":
    pointCloudDirectory = importFiles()
    object_features = np.array(allObjectProperties(pointCloudDirectory))
    
    #for testing
    #sampleFeatureList = [[0, 50,4,5], [1, 10, 5, 2], [2, 15,4,3], [3, 20,4,5]]

    hc.compare_clusters(object_features, -4)



    

    #temp data generation
    pointcloudsdummy = np.random.randn(800).reshape((100,8))

    # DBSCAN - dbscan(data, [features from data], epsilon(radius distance), min number of points in cluster)
    cluster, dataConsidered = DBSCAN.dbscan(pointcloudsdummy, [0,1,3,4,5], 1.3, 3)

    cc.cluster_accuracy(cluster)


    # K-means - kMeans(data, k-clusters, [features from dataset])
    cluster2, centroids, dataConsidered = KMeans.kMeans(pointcloudsdummy,5, [0,1,2,4,5])

    cc.cluster_accuracy(cluster2)
