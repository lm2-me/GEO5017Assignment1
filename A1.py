#Github
''' git status, git add ., git commit -m "comment about update", git push. --> git pull'''

#Import Point Cloud
import numpy as np
import open3d as o3d
import math as m

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
    pc_names = []
    for i in range(500):
        # print(i)
        number = "00" + str(i)
        three_digit = number[-3:]
        open_array = np.genfromtxt("{0}.xyz".format(three_digit))
        d["pc{0}".format(three_digit)] = open_array
        pc_names.append("pc{0}".format(three_digit))
    
    cwd = os.getcwd()
    filewd = (cwd[: len(cwd) - 11])
    #print('file wd' + filewd)
    save_pc = filewd + 'pointclouds.txt'

    with open (save_pc, 'w') as f:
        for i in range(500):
            f.write(str(d[pc_names[i]]) + "\n")
    f.close()

    #print(d["pc385"])
    return d

#visualize point cloud
def visualizePC(pc):
    cloud = currento3dPCfile(pc)
    o3d.visualization.draw_geometries([cloud])

#current o3d point cloud gets the file for open3d to use in visualizations
def currento3dPCfile(pc):

    number = "00" + str(pc)
    three_digit = number[-3:] 
    filewd = os.getcwd()
    folder = "{0}\{1}.xyz".format(filewd, three_digit)

    currentPointCloud = o3d.io.read_point_cloud(folder)
    return currentPointCloud

#Get object features for each point cloud height
def allObjectProperties(pointCloudDirectory):
    i = 0
    object_features = []
    print('Evaluating point cloud features')
    for pc in pointCloudDirectory:
        print ('now working on point cloud', str(pc), end="\r")
        #Get current point cloud
        currentPointCloud = currentPC(pointCloudDirectory, pc)
        currentPointCloud_o3d = currento3dPCfile(pc)

        #Visualize Point Cloud
        #visualizePC(pc)

        #Get properties by calling related function, only getting 3 best features after analysis of all features
        height = objectHeight(currentPointCloud)
        #volume = convexHull(currentPointCloud_o3d)
        avg_height = objectAverageHeight(currentPointCloud)
        #area, ratio = areaBase(currentPointCloud_o3d)
        num_planes = planarityPC(currentPointCloud_o3d)

        #object_features.append([i, height, volume, avg_height, area, ratio, num_planes])
        object_features.append([i, height, avg_height, num_planes])
        
        i += 1
        #print(str(i) + " height: " + str(height) + " volume: " + str(volume) + " average height: " + str(avg_height) + " area: " + str(area) + " ratio: " + str(ratio) + " num planes: " + str(num_planes))

    return object_features

#Get current point cloud and save to new array
def currentPC(pointCloudDirectory, pc):
    number = "00" + str(pc)
    three_digit = number[-3:]
    name = "pc" + three_digit
    currentPointCloud = pointCloudDirectory[name]
    return currentPointCloud

#normalize  to put into range from 0 to 1
def normalize_features(object_features):
    print('Normalizing point cloud features')
    all_normalized_features = np.copy(object_features)
    for i in range(1, object_features.shape[1]):
        min = np.min(object_features[:,i])
        normalized_feature = object_features[:,i] - min
        max = np.max(normalized_feature)
        normalized_feature = normalized_feature / max
        all_normalized_features[:,i] = normalized_feature
    return all_normalized_features

#Get feature 1: Height
def objectHeight(currentPointCloud):
    maxZ = 0

    for point in currentPointCloud:
        if point[2] > maxZ:
            maxZ = point[2]
        #maxZ now largest Z

    height = maxZ
    return height

#Feature 2: Convex hull for Volume
def convexHull(pc):
    convhull, _ = pc.compute_convex_hull()
    convhull_lns = o3d.geometry.LineSet.create_from_triangle_mesh(convhull)
    convhull_lns.paint_uniform_color((0, 0, 1))

    #visualize convex hull
    #o3d.visualization.draw_geometries([pc, convhull_lns])
    
    volume = convhull.get_volume()
    return volume

#Feature 3: planarity
def planarityPC(pc):
    allpoints = pc
    numplanes = 0
    #get planes with o3d segment_plane
    
    pcarray = np.asarray(pc.points)
    numpoints = len(pcarray)

    #accuracy
    p = 0.99
    #error rate
    e = 0.9
    #percision level
    s = 3
    #iterations
    n = int(m.ceil(m.log(1 - p) / m.log(1 - m.pow(1 - e, s))))

    while len(np.asarray(allpoints.points)) > .2 * len(pcarray):
        indexes = []
        
        plane_model, inliers = allpoints.segment_plane(distance_threshold=0.1,ransac_n=3, num_iterations=n)

        inlier_cloud = allpoints.select_by_index(inliers)


        for item in range(len(np.asarray(allpoints.points))):
            if item in inliers:
                continue
            else:
                indexes.append(item)

        allpoints = allpoints.select_by_index(indexes)
        numplanes += 1

        #visualize planes
        #inlier_cloud.paint_uniform_color([1.0, 0, 0])
        #allpoints.paint_uniform_color([0, 0, 1])
        #o3d.visualization.draw_geometries([allpoints, inlier_cloud])

    return numplanes

#Get feature 4: Average Height
def objectAverageHeight(currentPointCloud):
    
    npCurrentPointCloud = np.array(currentPointCloud)
    allHeights = npCurrentPointCloud[:,2]

    averageHeight = sum(allHeights) / len(allHeights)

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
    ratio = length / width
    
    bBox.color = (0, 0, 1)

    #visualize bounding box
    #o3d.visualization.draw_geometries([pc, bBox])
    return area, ratio

#function we had used for checking all possible combinations of three features, not used in final assignment
def get_best_features(normalized_object_features):
    feature_combos = [[1, 2, 3],
    [1, 2, 4],
    [1, 2, 5],
    [1, 2, 6],
    [1, 3, 4],
    [1, 3, 5],
    [1, 3, 6],
    [1, 4, 5],
    [1, 4, 6],
    [1, 5, 6],
    [2, 3, 4],
    [2, 3, 5],
    [2, 3, 6],
    [2, 4, 5],
    [2, 4, 6],
    [2, 5, 6],
    [3, 4, 5],
    [3, 4, 6],
    [3, 5, 6],
    [4, 5, 6]
    ]

    for i in feature_combos:
        column_indexes = np.concatenate(([0], i))
        feature_test = normalized_object_features[:,column_indexes]

        hc.compare_clusters(feature_test, -4) 

        cluster, dataConsidered = DBSCAN.dbscan(feature_test, [1,2,3], 1.3, 3)
        print('DBSCAN accuracy')
        cc.cluster_accuracy(cluster)

        cluster2, centroids, dataConsidered = KMeans.kMeans(feature_test, 5, [1,2,3])
        print('k-means accuracy')
        cc.cluster_accuracy(cluster2)

        print('\n\n')
        print('===========================')


#Main
if __name__ == "__main__":
    pointCloudDirectory = importFiles()
    object_features = np.array(allObjectProperties(pointCloudDirectory))
    normalized_object_features = normalize_features(object_features)

    #not used in final assignment, function to get best features
    #get_best_features(normalized_object_features)

    feature_indexes = range(1, len(normalized_object_features[0]))

    # Hierarchical compare_clusters(data, height to cut dendograph at (one less than number of clusters))
    npFeatureList, cut_link, cluster_table, height, title = hc.compare_clusters(normalized_object_features, -4)  

    # DBSCAN - dbscan(data, [features from data], epsilon(radius distance), min number of points in cluster)
    cluster, dataConsidered = DBSCAN.dbscan(normalized_object_features, feature_indexes, 1.3, 3)
    print('DBSCAN accuracy')
    cc.cluster_accuracy(cluster)


    # K-means - kMeans(data, k-clusters, [features from dataset])
    cluster2, centroids, dataConsidered = KMeans.kMeans(normalized_object_features, 5, feature_indexes)
    print('k-means accuracy')
    cc.cluster_accuracy(cluster2)

    #Visualizations
    #hc.visualize_hierarchy(npFeatureList, cut_link, cluster_table, height)
    #print(title + 'Hierarchical Clustering Visualized')


