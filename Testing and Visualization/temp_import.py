import numpy as np
import math
import random
import matplotlib.pyplot as plt

from xml.dom import WrongDocumentErr
import pandas as pd
import numpy as np
# import pyoctree as poct
import open3d as o3d
import math
import os
import copy

import ClusterComparing as CC
import DBSCAN as DBSCAN
import KMeans



pointcloudsdummy = np.random.randn(800).reshape((100,8))

cluster, dataConsidered = DBSCAN.dbscan(pointcloudsdummy, [0,1,3,4,5], 1.3, 3)

CC.cluster_accuracy(cluster)

cluster2, centroids, dataConsidered = KMeans.kMeans(pointcloudsdummy,5, [0, 1,2,4,5])

CC.cluster_accuracy(cluster2)