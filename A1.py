


#Import Point Cloud

#Get object height for each point cloud height
def allObjectProperties(pointCloudDirectory):
    for file in pointCloudDirectory:
        #read file
        #assign data from current file to currentPointCloud variable
        currentPointCloud = [[1,4,8], [3,9,5], [2,2,7]]
        height = objectHeight(currentPointCloud)
        bBox = boundingBox(currentPointCloud, height)

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

#Create Bounding Box
def boundingBox(currentPointCloud, height):
    maxX = 0
    minX = 100
    maxY = 0
    minY = 100

    for point in currentPointCloud:
        if point[0] < minX:
            minX = point[0]
        if point[0] > maxX:
            maxX = point[0]
        if point[1] < minY:
            minY = point[1]
        if point[1] > maxY:
            maxY = point[1]
        #minX, minY, maxX, maxY now defined
    
    minCorner = (minX, minY, 0)
    maxCorner = (maxX, maxY, height)

    bBox = (minCorner, maxCorner)
    
    print(bBox)
    return bBox

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
    allObjectProperties(pointCloudDirectory)