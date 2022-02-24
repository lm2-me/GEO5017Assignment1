import math
import numpy as np


def distance2pts(vector1, vector2, p=2):
    #Euclidian p = 2
    #Manhatten p = 1
    num = 0
    for i in range(0,len(vector1)):
        num += abs(vector1[i]-vector2[i]) ** p
    return num ** (1/p)


#distance2pts([0,2,3,4],[2,4,3,7],1)

a = np.array((1.1, 2.2, 3.3))
b = np.array((4.4, 5.5, 6.6))

#distance2pts(a,b,1.5)

#implement a seed?

# start positions


# def distEuclidian()

#loop
#compute closest

pointcloudsdummy = np.random.randint(0,400,75).reshape((25,3))

#pointcloudsdummy = np.array([[2,5,4],[48,89,6],[4,3,8],[4,2,8],[7,5,6]])

def kMeans(data, k, feature1, feature2):
    #cluster = np.zeros(shape=[500,2]) #used for?
    dataset_length = len(data)
    print("Dataset length is: ", dataset_length)

    dataConsidered = data[:,[feature1,feature2]]
    #print(dataConsidered)

    pick_centroids = np.random.randint(0,dataset_length,k)
    print(pick_centroids)
    
    #print("datacon", dataConsidered)

    print("this is k: ", k)

    centroids = np.empty([k,2])
    print("start centroid set: ", centroids)

    for i, c in enumerate(pick_centroids):
        print(c)
        #centroids = np.append(centroids, [dataConsidered[c]])
        centroids[i] = dataConsidered[c]

    print("these are the centroids ", centroids)


    clustergroup = np.zeros(dataset_length)
    clustergroup_new = np.zeros(dataset_length)
    
    running = 1
    while running == 1:
        print("start while loop")
        #for each point:
        for i, location in enumerate(dataConsidered):
            print("level 1 loop")
            distance = float('inf') # our distance from each point to centroid
            #now check for each centroid
            for i2, centroid in enumerate(centroids):
                print("level 2 loop")
                temp_dist = distance2pts(location, centroid)
                if temp_dist < distance:
                    distance = temp_dist
                    clustergroup_new[i] = centroid
            print("cluster group subtraction, old from new: ", clustergroup - clustergroup_new)

            if clustergroup - clustergroup_new != 0:
                clustergroup = clustergroup_new
            else:
                running = 0


        #update centroids data
        #take all items in dictionary with that centroids, then calculate new center, then update centroid data
        for i, centroid in enumerate(centroids):
            temp_list = []
            for i2, item in clustergroup:
                if item == centroid:
                    temp_list.append(dataConsidered[i2])

            centroid[i] = temp_list.mean()


#dictionary where each is has number of cluster, 000: 1, 001:3, 002: 3, etc

kMeans(pointcloudsdummy,3, 0, 1)

# print(pointcloudsdummy)

#compute centroids

#move K;s to centroids

