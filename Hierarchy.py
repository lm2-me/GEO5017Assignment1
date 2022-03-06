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

#distance matrix for average and single-link clusteirng
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

#distance matrix for complete clustering
def generate_distance_minmax(clusterList):
    labels = clusterList[:,0]
    max_pts = max([len(a) for a in clusterList[:, 1]])

    y_values = np.array([np.pad(a, ((0, max_pts - len(a)), (0, 0)), 'edge') for a in clusterList[:, 1]])
    n = len(clusterList)
    f = len(clusterList[0, 1] [0])

    dist_table_1 = np.repeat(np.repeat(y_values, max_pts, axis=1), n, axis=0).reshape((n, n, max_pts, max_pts, f))
    dist_table_2 = dist_table_1.transpose(1, 0, 3, 2, 4)

    dist_table_diff = (dist_table_1 - dist_table_2) ** 2
    dist_table_full = np.sqrt(np.sum(dist_table_diff, axis=4))
    dist_table_full_max = np.max(np.max(dist_table_full, axis=3), axis=2)
    diagonal_multiplier = np.ones((n, n)) - np.eye(n, n)
    dist_table_nozero_no_diagnoal = dist_table_full_max * diagonal_multiplier

    dist_table_nozero = np.where(dist_table_nozero_no_diagnoal == 0, np.Infinity, dist_table_nozero_no_diagnoal)

    return dist_table_nozero, labels

#link clusters based on nearest points in clusters
def hierarchy_singlelink_clustering(npFeatureList):

    #remove 1st index which is storing the point cloud number
    fl = np.delete(npFeatureList, 0, 1)

    #compute distance matrix
    dist_matrix, labels = generate_distance_matrix(npFeatureList)

    dist_list = []
    for i in range(dist_matrix.shape[0]):
        for j in range(dist_matrix.shape[1]):
            if j >= i: continue
            dist_list.append((dist_matrix[i, j], [i, j]))
    
    #default sorting is by first value (which is the distance between points) so sort() is okay to use
    dist_list.sort()

    #initialize variables to track clustering
    point_current_cluster = np.array([[i] for i in np.arange(len(fl))])
    cluster_table = []
    next_cluster_id = len(fl)
    num_points = np.array([[i, 1] for i in np.arange(len(fl))])
    cluster_lookup = np.array([[i, i] for i in np.arange(len(fl))])
    points_in_cluster = {}

    #track which cluster points belong to and the number of points in each cluster
    for dist, pts in dist_list:
        cl1 = cluster_lookup[pts[0], 1]
        cl2 = cluster_lookup[pts[1], 1]

        if cl1 == cl2: continue
        total_points = num_points[cl1, 1] + num_points[cl2, 1]
        cluster_table.append([cl1, cl2, dist, total_points])
        num_points = np.append(num_points, [[next_cluster_id, total_points]], axis=0)
        
        append_cluster = []

        for p in range(len(cluster_lookup)):
            cluster = cluster_lookup[p, 1]
            append_cluster.append(cluster)
        
        point_current_cluster = np.append(point_current_cluster, np.array([append_cluster]).T, axis=1)
        
        for i in range(len(cluster_lookup)):
            if cluster_lookup[i, 1] == cl1 or cluster_lookup[i, 1] == cl2:
                cluster_lookup[i, 1] = next_cluster_id
                if next_cluster_id not in points_in_cluster:
                    points_in_cluster[next_cluster_id] = []
                points_in_cluster[next_cluster_id].append(i)
        next_cluster_id += 1

    return point_current_cluster, cluster_table

#link clusters based on average distance between points in clusters
def hierarchy_avglink_clustering(npFeatureList):

    currentPoints = np.copy(npFeatureList)
    point_current_cluster = np.array([[f[0]] for f in npFeatureList])
    cluster_table = []
    next_cluster_id = len(npFeatureList)
    num_points = np.array([[f[0], 1] for f in npFeatureList])
    cluster_lookup = np.array([[f[0], f[0]] for f in npFeatureList])
    points_in_cluster = {}

    #get 1st distance matrix
    while len(currentPoints) > 1:
        print ('now working on cluster', next_cluster_id-len(npFeatureList), end="\r")
        dist_matrix, labels = generate_distance_matrix(currentPoints)

        dist_list = []
        for i in range(dist_matrix.shape[0]):
            for j in range(dist_matrix.shape[1]):
                if j >= i: continue
                l1 = labels[i]
                l2 = labels[j]
                dist_list.append((dist_matrix[i, j], [l1, l2]))
        
        dist_list.sort()
        dist = dist_list[0] [0]
        cl1 = int(dist_list[0] [1] [0])
        cl2 = int(dist_list[0] [1] [1])

        if cl1 == cl2: continue

        total_points = num_points[cl1, 1] + num_points[cl2, 1]
        cluster_table.append([cl1, cl2, dist, total_points])  
        num_points = np.append(num_points, [[next_cluster_id, total_points]], axis=0)

        
        append_cluster = []

        #append point_current_cluster to add the number that it is currently in
        for p in range(len(cluster_lookup)):
            cluster = cluster_lookup[p, 1]
            append_cluster.append(cluster)

        point_current_cluster = np.append(point_current_cluster, np.array([append_cluster]).T, axis=1)

        #update cluster lookup to know which point is a part of which cluster
        for c in range(len(cluster_lookup)):
            if cluster_lookup[c, 1] == cl1 or cluster_lookup[c, 1] == cl2:
                cluster_lookup[c, 1] = next_cluster_id
                if next_cluster_id not in points_in_cluster:
                    points_in_cluster[next_cluster_id] = []
                points_in_cluster[next_cluster_id].append(cluster_lookup[c, 0])
        
        #get average distance of  new cluster
        all_points = []
        for pt in points_in_cluster[next_cluster_id]:
            all_points.append(npFeatureList[int(pt)][1:])

        for j in range(len(currentPoints))[::-1]:
            if currentPoints[j, 0] == cl1 or currentPoints[j, 0] == cl2: 
                currentPoints = np.delete(currentPoints, [j], 0)

        all_points = np.array(all_points)

        mean = [np.concatenate((np.array([next_cluster_id]), np.mean(all_points, axis=0)))]

        currentPoints = np.append(currentPoints, mean, 0)

        next_cluster_id += 1

    return point_current_cluster, cluster_table

#link clusters based on furthest points in clusters
def hierarchy_completelink_clustering(npFeatureList):
    currentPoints = np.array([[f[0], [f[1:]]] for f in npFeatureList], dtype=object)
    point_current_cluster = np.array([[f[0]] for f in npFeatureList])

    cluster_table = []
    next_cluster_id = len(npFeatureList)
    num_points = np.array([[f[0], 1] for f in npFeatureList])
    cluster_lookup = np.array([[f[0], f[0]] for f in npFeatureList])
    points_in_cluster = {}

    dist_matrix, labels = generate_distance_minmax(currentPoints)

    #get 1st distance matrix
    while len(currentPoints) > 1:
        print ('now working on cluster', next_cluster_id-len(npFeatureList), end="\r")

        dist_list = []
        for i in range(dist_matrix.shape[0]):
            for j in range(dist_matrix.shape[1]):
                if j >= i: continue
                l1 = labels[i]
                l2 = labels[j]
                dist_list.append((dist_matrix[i, j], [l1, l2]))
        
        dist_list.sort()

        dist = dist_list[0] [0]
        cl1 = int(dist_list[0] [1] [0])
        cl2 = int(dist_list[0] [1] [1])

        if cl1 == cl2: continue

        total_points = num_points[cl1, 1] + num_points[cl2, 1]
        cluster_table.append([cl1, cl2, dist, total_points])  
        num_points = np.append(num_points, [[next_cluster_id, total_points]], axis=0)

        append_cluster = []

        #append point_current_cluster to add the number that it is currently in
        for p in range(len(cluster_lookup)):
            cluster = cluster_lookup[p, 1]
            append_cluster.append(cluster)

        point_current_cluster = np.append(point_current_cluster, np.array([append_cluster]).T, axis=1)

        #update cluster lookup to know which point is a part of which cluster
        for c in range(len(cluster_lookup)):
            if cluster_lookup[c, 1] == cl1 or cluster_lookup[c, 1] == cl2:
                cluster_lookup[c, 1] = next_cluster_id
                if next_cluster_id not in points_in_cluster:
                    points_in_cluster[next_cluster_id] = []
                points_in_cluster[next_cluster_id].append(cluster_lookup[c, 0])

        all_points = []
        for pt in points_in_cluster[next_cluster_id]:
            all_points.append(npFeatureList[int(pt)][1:])

        for j in range(len(currentPoints))[::-1]:
            if currentPoints[j, 0] == cl1 or currentPoints[j, 0] == cl2: 
                currentPoints = np.delete(currentPoints, [j], 0)

        new_cluster = [[next_cluster_id, all_points]]
        currentPoints = np.append(currentPoints, new_cluster, 0)

        # Update dist table

        # Get indexes of clusters from label ids
        cl1_ix = np.where(labels == cl1)[0][0]
        cl2_ix = np.where(labels == cl2)[0][0]

        # Compute new max col/row for combined cluster
        cl_max_col = np.max(dist_matrix[:, [cl1_ix, cl2_ix]], axis=1).reshape((dist_matrix.shape[0], 1))
        cl_max_row = np.concatenate((np.max(dist_matrix[[cl1_ix, cl2_ix], :], axis=0), [np.Infinity]))

        # Add col, then row to dist matrix for new combined cluster
        dist_matrix = np.hstack((dist_matrix, cl_max_col))
        dist_matrix = np.vstack((dist_matrix, [cl_max_row]))

        # Remove cols, then rows corresponding to old clusters
        dist_matrix = np.delete(dist_matrix, [cl1_ix, cl2_ix], 1)
        dist_matrix = np.delete(dist_matrix, [cl1_ix, cl2_ix], 0)

        # Update labels array
        labels = np.concatenate((labels, [next_cluster_id]))
        labels = np.delete(labels, [cl1_ix, cl2_ix])

        next_cluster_id += 1

    return point_current_cluster, cluster_table

#compare accuracy of clusters
def compare_clusters(npFeatureList, height):
    single_link, sl_cluster_table = hierarchy_singlelink_clustering(npFeatureList)
    print('single link clustering done')
    avg_link, avg_cluster_table = hierarchy_avglink_clustering(npFeatureList)
    print('average link clustering done')
    complete_link, compl_cluster_table = hierarchy_completelink_clustering(npFeatureList)
    print('complete link clustering done')

    cut_single_link = single_link[:,(height)]
    cut_avg_link = avg_link[:,(height)]
    cut_comp_link = complete_link[:,(height)]

    print('single link clustering accuracy')
    acc_single = cc.cluster_accuracy(cut_single_link)
    print('average link clustering accuracy')
    acc_avg = cc.cluster_accuracy(cut_avg_link)
    print('complete link clustering accuracy')
    acc_complete = cc.cluster_accuracy(cut_comp_link)

    accuracy_counts = [acc_single, acc_avg, acc_complete]

    max_id = np.argmax(accuracy_counts)

    if max_id == 0:
        #visualize_hierarchy(npFeatureList, cut_single_link, sl_cluster_table, height)
        return npFeatureList, cut_single_link, sl_cluster_table, height, 'Single-Link'
    elif max_id == 1:
        #visualize_hierarchy(npFeatureList, cut_avg_link, avg_cluster_table, height)
        return npFeatureList, cut_avg_link, avg_cluster_table, height, 'Average-Link'
    elif max_id == 2:
        #visualize_hierarchy(npFeatureList, cut_comp_link, compl_cluster_table, height)
        return npFeatureList, cut_comp_link, compl_cluster_table, height, 'Complete-Link'

#color code points based on which cluster they are in
def color_coded_cluster(clusters_at_cut_height, height):

    unique_cluster_nums = np.unique(clusters_at_cut_height)
    color_num = 0

    for i in range(len(unique_cluster_nums)):
        clusters_at_cut_height = np.where(clusters_at_cut_height==unique_cluster_nums[i], color_num, clusters_at_cut_height)
        color_num += 1

    return clusters_at_cut_height 
    
#graph showing best hierachical clustering
def visualize_hierarchy(npFeatureList, clusters_at_cut_height, cluster_table, height):
    #visualize clusters
    colors = color_coded_cluster(clusters_at_cut_height, height)
    plt.figure(figsize=(10, 7)) 
    ax = plt.axes(projection='3d')
    plt.title('Group 5 Hierarchical Clustering Scatter Plot')
    ax.set_xlabel('No. Planes')
    ax.set_ylabel('Average Height')
    ax.set_zlabel('Height')
    ax.scatter(npFeatureList[:,3], npFeatureList[:,2], npFeatureList[:,1], s=40, c=colors) 


    #visualize dendrogram of clustering
    cut_height = cluster_table[height][2] + .05
    visualize_dendrogram(cluster_table, cut_height, clusters_at_cut_height)
    return cluster_table

#dendrogram showing best hierarchical clustering
def visualize_dendrogram(cluster_table, cut_height, clusters_at_cut_height):
    plt.figure()
    plt.title('Group 5 Hierarchical Clustering Dendrogram')
    plt.xlabel('point cloud')
    plt.ylabel('distance')

    
    dendrogram(cluster_table, leaf_rotation=90., leaf_font_size=8.)
    plt.axhline(y=cut_height)
    plt.show()
