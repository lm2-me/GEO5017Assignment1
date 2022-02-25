
from xml.dom import WrongDocumentErr
import pandas as pd
import numpy as np
# import pyoctree as poct
import open3d as o3d
import math
import os
#only used to visualize dendrogram after our own implementation of the hierarchy clustering
from scipy.cluster.hierarchy import dendrogram
import matplotlib.pyplot as plt

import Hierarchy as hc

"""000 - 099: building
100 - 199: car
200 - 299: fence
300 - 399: pole
400 - 499: tree"""

def cluster_accuracy(cluster):
    points_labeled = []
    alg_clstr_label_options = {}
    alg_clstr_label = {}
    label_list = ['building', 'car', 'fence', 'pole', 'tree']
    count_correct = 0
    count_incorrect = 0

    for i in range(len(cluster)):
        clusternum = cluster[i]
        actual_label = string_label(i)
        if clusternum not in alg_clstr_label_options:
            alg_clstr_label_options[clusternum] = []
        alg_clstr_label_options[clusternum].append(actual_label)
    
    for c in alg_clstr_label_options:
        options = np.array(alg_clstr_label_options[c])
        label_counts = np.array([sum(np.where(options == l, 1, 0)) for l in label_list])
        max_id = np.argmax(label_counts)
        alg_clstr_label[c] = label_list[max_id]
        #print('label counts', label_counts)
    

    #print('alg cluster', alg_clstr_label)
    #print('alg cluster options', alg_clstr_label_options)

    for i in range(len(cluster)):
        clusternum = cluster[i]
        actual_label = string_label(i)
        alg_label = alg_clstr_label[clusternum]
        points_labeled.append((i, clusternum, actual_label, alg_label))
        if alg_label == actual_label:
            count_correct += 1

        else:
            count_incorrect += 1

    percent_correct = count_correct / (count_correct + count_incorrect)
    #print('labeled', points_labeled)
    print('correct', count_correct)
    print('incorrect', count_incorrect)
    print('percentage', percent_correct)
    return percent_correct




def string_label(ptcld):
    if ptcld < 100:
        #building
        return "building"

    elif ptcld >= 100 and ptcld < 200:
        #car
        return "car"
        
    elif ptcld >= 200 and ptcld < 300:
        #fence
        return "fence"

    elif ptcld >= 300 and ptcld < 400:
        #pole
        return "pole"
    
    elif ptcld >= 400 and ptcld < 500:
        #tree
        return "tree"