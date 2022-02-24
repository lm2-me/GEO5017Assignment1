
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


def cluster_accuracy(cluster):
    a=1