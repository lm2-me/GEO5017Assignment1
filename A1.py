


#Import Point Cloud

#Get object height for each point cloud height
def allObjectHeights(pointCloudDirectory):
    for file in pointCloudDirectory:
        #read file
        #assign data from current file to currentPointCloud variable
        currentPointCloud = [[1,4,8], [3,4,5], [2,4,7]]
        objectHeight(currentPointCloud)

#Get feature 1: Height
def objectHeight(currentPointCloud):
    maxZ = 0
    length = len(currentPointCloud)

    for point in currentPointCloud:
        if point[2] > maxZ:
            maxZ = point[2]
        #maxZ now largest Z

    height = maxZ
    print(height)
    return height

#Get feature 2: Vertical Slices


#Get feature 3

#store feature data

#plot features against each other

#implement clustering

#Main
if __name__ == "__main__":
    #function importing point clouds
    # assign to pointCloudDirectory variable 
    pointCloudDirectory = [" "]
    allObjectHeights(pointCloudDirectory)