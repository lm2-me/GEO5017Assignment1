import pandas as pd
import numpy as np
# import pyoctree as poct
import open3d as o3d
import math
import os
#only used to visualize dendrogram after our own implementation of the hierarchy clustering
from scipy.cluster.hierarchy import dendrogram
import matplotlib.pyplot as plt

import ClusterComparing as cc


#Hierachy Clusturing
#Create distance matrix

def generate_distance_matrix(npFeatureList):
    fl = np.delete(npFeatureList, 0, 1)
    labels = npFeatureList[:,0]
    n, m = fl.shape[:2]
    p_repeated = np.repeat([fl],n, axis=1).reshape((n, n, m))
    p_repeated_T = p_repeated.transpose((1, 0, 2))

    points_sq = (p_repeated - p_repeated_T) ** 2
    points_sq_sum = np.sum(points_sq, axis=2)
    points_sq_sum_rt = np.sqrt(points_sq_sum)

    return points_sq_sum_rt, labels


def hierarchy_singlelink_clustering(npFeatureList):

    #remove 1st index of all lists
    pcnum = npFeatureList[:,0]
    fl = np.delete(npFeatureList, 0, 1)
    #print('single fl', fl)

    #compute distance matrix
    dist_matrix, labels = generate_distance_matrix(npFeatureList)
    #print('single dist matrix', dist_matrix)
    
    #single-linkage clustering
    dist_list = []
    for i in range(dist_matrix.shape[0]):
        for j in range(dist_matrix.shape[1]):
            if j >= i: continue
            dist_list.append((dist_matrix[i, j], [i, j]))
    
    #print('distlist', dist_list)
    #default sorting is by first value (which is the distance between points) so sort() is okay to use
    dist_list.sort()
    #print(dist_list)

    #initialize variables to track clustering
    point_current_cluster = np.array([[i] for i in np.arange(len(fl))])
    #print('point current cluster', point_current_cluster)
    cluster_table = []
    next_cluster_id = len(fl)
    num_points = np.array([[i, 1] for i in np.arange(len(fl))])
    cluster_lookup = np.array([[i, i] for i in np.arange(len(fl))])
    points_in_cluster = {}
    height = -5

    #track which cluster points belong to and the number of points in each cluster
    for dist, pts in dist_list:
        #print("pts", pts)
        #print("dist", dist)

        cl1 = cluster_lookup[pts[0], 1]
        cl2 = cluster_lookup[pts[1], 1]
        #print("cl", cl1, cl2)
        if cl1 == cl2: continue
        total_points = num_points[cl1, 1] + num_points[cl2, 1]
        cluster_table.append([cl1, cl2, dist, total_points])
        num_points = np.append(num_points, [[next_cluster_id, total_points]], axis=0)

        
        append_cluster = []

        for p in range(len(cluster_lookup)):
            cluster = cluster_lookup[p, 1]
            append_cluster.append(cluster)
            #print ('append cluster', append_cluster)
        
        point_current_cluster = np.append(point_current_cluster, np.array([append_cluster]).T, axis=1)
        #print('updated current cluster', point_current_cluster)
        
        for i in range(len(cluster_lookup)):
            if cluster_lookup[i, 1] == cl1 or cluster_lookup[i, 1] == cl2:
                cluster_lookup[i, 1] = next_cluster_id
                if next_cluster_id not in points_in_cluster:
                    points_in_cluster[next_cluster_id] = []
                points_in_cluster[next_cluster_id].append(i)
        #print("points in cluster", points_in_cluster)
        next_cluster_id += 1
    #print("table", cluster_table)
    #print("lookup", cluster_lookup)

    return point_current_cluster, cluster_table

    

def hierarchy_avglink_clustering(npFeatureList):
    #remove 1st index of all lists
    pcnum = npFeatureList[:,0]
    #fl = np.delete(npFeatureList, 0, 1)
    #print('fl', fl)

    currentPoints = np.copy(npFeatureList)
    point_current_cluster = np.array([[f[0]] for f in npFeatureList])
    #print('point current cluster', point_current_cluster)
    cluster_table = []
    next_cluster_id = len(npFeatureList)
    num_points = np.array([[f[0], 1] for f in npFeatureList])
    cluster_lookup = np.array([[f[0], f[0]] for f in npFeatureList])
    points_in_cluster = {}

    #get 1st distance matrix
    while len(currentPoints) > 1:
        #print('currentpoints', currentPoints)
        dist_matrix, labels = generate_distance_matrix(currentPoints)
        #print(dist_matrix)

        dist_list = []
        for i in range(dist_matrix.shape[0]):
            for j in range(dist_matrix.shape[1]):
                if j >= i: continue
                l1 = labels[i]
                l2 = labels[j]
                dist_list.append((dist_matrix[i, j], [l1, l2]))
        
        dist_list.sort()
        #print(dist_list)

        dist = dist_list[0] [0]
        cl1 = int(dist_list[0] [1] [0])
        cl2 = int(dist_list[0] [1] [1])
        #print('dist', dist)
        #print("cl", cl1, cl2)

        if cl1 == cl2: continue

        total_points = num_points[cl1, 1] + num_points[cl2, 1]
        cluster_table.append([cl1, cl2, dist, total_points])  
        num_points = np.append(num_points, [[next_cluster_id, total_points]], axis=0)

        
        append_cluster = []

        #append point_current_cluster to add the number that it is currently in
        for p in range(len(cluster_lookup)):
            cluster = cluster_lookup[p, 1]
            append_cluster.append(cluster)
        #print('append', append_cluster)
        point_current_cluster = np.append(point_current_cluster, np.array([append_cluster]).T, axis=1)

        #update cluster lookup to know which point is a part of which cluster
        for c in range(len(cluster_lookup)):
            if cluster_lookup[c, 1] == cl1 or cluster_lookup[c, 1] == cl2:
                cluster_lookup[c, 1] = next_cluster_id
                if next_cluster_id not in points_in_cluster:
                    points_in_cluster[next_cluster_id] = []
                points_in_cluster[next_cluster_id].append(cluster_lookup[c, 0])
            #print("points in cluster", points_in_cluster)
        
        #get average distance of  new cluster
        #print ('currentpoints here', currentPoints)
        all_points = []
        for pt in points_in_cluster[next_cluster_id]:
            all_points.append(npFeatureList[int(pt)][1:])

        for j in range(len(currentPoints))[::-1]:
            #print('j', j, pt)
            if currentPoints[j, 0] == cl1 or currentPoints[j, 0] == cl2: 
                currentPoints = np.delete(currentPoints, [j], 0)
            #print ('point', pt)
            #print('index pt', currentPoints[pt])

        all_points = np.array(all_points)
        mean = np.array([[next_cluster_id, np.mean(all_points[:,0]), np.mean(all_points[:,1])]])
        #print('mean', mean)

        currentPoints = np.append(currentPoints, mean, 0)

        #print(currentPoints)

        next_cluster_id += 1

    return point_current_cluster, cluster_table


def compare_clusters(npFeatureList, height):
    single_link, sl_cluster_table = hierarchy_singlelink_clustering(npFeatureList)
    avg_link, avg_cluster_table = hierarchy_avglink_clustering(npFeatureList)

    cut_single_link = single_link[:,(height)]
    cut_avg_link = avg_link[:,(height)]

    #print('cut avg link', cut_avg_link)
    #print('cut single link', cut_single_link)

    acc_single = cc.cluster_accuracy(cut_single_link)
    acc_avg = cc.cluster_accuracy(cut_avg_link)
    acc_complete = 0

    accuracy_counts = [acc_single, acc_avg, acc_complete]

    max_id = np.argmax(accuracy_counts)

    if max_id == 0:
        visualize_heirarchy(npFeatureList, cut_single_link, sl_cluster_table, height)
        return cut_single_link, acc_single
    elif max_id == 1:
        visualize_heirarchy(npFeatureList, cut_avg_link, avg_cluster_table, height)
        return cut_avg_link, acc_avg
    """elif max_id == 2:
        visualize_heirarchy(npFeatureList, cut_single_link, sl_cluster_table, height)
        return cut_single_link, acc_single"""


def color_coded_cluster(clusters_at_cut_height, height):
    #print(clusters_at_cut_height)
    unique_cluster_nums = np.unique(clusters_at_cut_height)
    #print(unique_cluster_nums[0])

    color_num = 0

    for i in range(len(unique_cluster_nums)):
        clusters_at_cut_height = np.where(clusters_at_cut_height==unique_cluster_nums[i], color_num, clusters_at_cut_height)
        color_num += 1

    #print('revised cluster num', clusters_at_cut_height)
    return clusters_at_cut_height 
    
def visualize_heirarchy(npFeatureList, clusters_at_cut_height, cluster_table, height):
    #visualize clusters
    colors = color_coded_cluster(clusters_at_cut_height, height)
    plt.figure(figsize=(10, 7))  
    plt.scatter(npFeatureList[:,1], npFeatureList[:,2], c=colors) 


    #visualize dendrogram of clustering
    cut_height = cluster_table[height][2] + .05
    #cluster_id = len(cluster_table)  - 6
    #clpts = points_in_cluster[cluster_id]
    #print(clpts)
    visualize_dendrogram(cluster_table, cut_height)
    return cluster_table

def visualize_dendrogram(cluster_table, cut_height):
    plt.figure()
    dendrogram(cluster_table)
    #LM figure out how to make line dashed
    plt.axhline(y=cut_height, color='r')
    plt.show()
