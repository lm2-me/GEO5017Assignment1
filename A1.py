#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import pandas as pd
import numpy as np
import pyoctree as poct
import open3d as o3d
import math as m

import Hierarchy as hc
import ClusterComparing as cc
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
    
    number = "001"
    #number = "00" + str(pc)
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

        if i >=1: break

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
    allpoints = pc
    numplanes = 0
    indexes = []
    #get planes with o3d segment_plane
    
    pcarray = np.asarray(pc.points)

    for p in range(5):
        plane_model, inliers = allpoints.segment_plane(distance_threshold=0.1,ransac_n=3, num_iterations=1000)

        inlier_cloud = pc.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        o3d.visualization.draw_geometries([inlier_cloud])
        #print('inlier cloud', inlier_cloud)
        #print("pcarray", inliers)
        for item in range(len(pcarray)):
            if item in inliers:
                print('in list', item)
                continue
            else:
                indexes.append(item)
        #print('indexes', indexes)

        allpoints = allpoints.select_by_index(indexes)

        if len(inliers) > .2 * len(pcarray):
            numplanes += 1

        #print('inliers', inliers)
        o3d.visualization.draw_geometries([allpoints])

    #get planes with normals
    # downsampledpc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=.5, max_nn=30))
    
    # print(downsampledpc.normals[0])
    
    # normals = np.asarray(downsampledpc.normals)
    # print('num normals', len(normals))
    # planes = {}

    # for i in range(len(normals)):
    #     for j in range(i+1, len(normals), 1):
    #         dot = np.dot(normals[i], normals[j])
    #         if dot > 0.85 and dot <= 1:
    #             if i not in planes:
    #                 planes[i] = []
    #                 planes[i].append(normals[i])
    #             planes[i].append(normals[j])
    #             np.delete(normals, j)

    #             if j == len(normals):
    #                 np.delete(normals, i)

    # points_in_plane = []
    # print('length planes', len(planes))
    # for p in planes:
    #     points_in_plane.append(len(planes[p]))

    # print('points in plane', points_in_plane)

    # print('planes', len(planes[0]))
    # print('dot product', planes)
    #print('normals', allnormals)
    #downsampledpc.estimate_covariances(downsampledpc, search_param=o3d.geometry.KDTreeSearchParamKNN with knn = 30)
    #cos_angle = (vec1[0] * vec2[0] + vec1[1] * vec2[1]) / math.sqrt((vec1[0]**2 + vec1[1]**2) * (vec2[0]**2 + vec2[1]**2))
    #o3d.visualization.draw_geometries([downsampledpc], point_show_normal=True)
    

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

    #hc.compare_clusters(object_features, -4)



    
