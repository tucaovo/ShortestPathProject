import math
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString

def distanceBetweenPoints(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def dijkstra(matrix, start, end):
    distances = {i: math.inf for i in range(len(matrix))}
    distances[start] = 0
    unvisited = set(range(len(matrix)))
    prev = {}

    while unvisited:
        current = min(unvisited, key=lambda x: distances[x])
        if current == end:
            break
        unvisited.remove(current)

        for i, distance in enumerate(matrix[current]):
            if distance == -1:
                continue
            new_distance = distances[current] + distance
            if new_distance < distances[i]:
                distances[i] = new_distance
                prev[i] = current

    path = []
    current = end
    while current in prev:
        path.append(current)
        current = prev[current]
    path.append(start)
    path.reverse()

    return path

def checkIfLineIsInsidePolygon(currentValue, point, points, pointsMock):
    index=0
    point_a = Point(currentValue)
    point_b = Point(point)
    lineToCheck = LineString([point_a, point_b])
    xvalue1=currentValue[0]+point[0]
    xvalue1=xvalue1/2
    yvalue1=currentValue[1]+point[1]
    yvalue1=yvalue1/2
    point_c=(xvalue1,yvalue1)
    lineToCheckMock=LineString([point_a,point_c])
    polygon = Polygon(points)
    if polygon.contains(lineToCheck):
        if polygon.contains(lineToCheckMock):
            index=1
    for i in range(0, len(pointsMock) - 2):
        point_a1 = Point(pointsMock[i])
        point_b1 = Point(pointsMock[i + 1])
        pretenderLine = LineString([point_a1, point_b1])
        if lineToCheck.equals(pretenderLine):
            index=1
    if index==0:
        return False
    else:
        return True

def triangluation(currentValue, matrixOfValues, points, pointsMock):
    values = []
    values.clear()
    for point in pointsMock:
        if checkIfLineIsInsidePolygon(currentValue, point, points, pointsMock):
            values.append(distanceBetweenPoints(currentValue[0], currentValue[1], point[0], point[1]))
        else:
            if currentValue == point:
                values.append(0)
            else:
                values.append(-1)
    matrixOfValues.append(values)

def shortestPath(start, end, points):
    polygon = points.copy()
    polygon.append(polygon[0])
    pointsMock = []
    pointsMock.append(start)
    for edges in points:
        pointsMock.append(edges)
    pointsMock.append(end)
    currentValue = start
    matrixOfValues = []
    values = []
    triangluation(currentValue, matrixOfValues, points, pointsMock)
    for edges in points:
        currentValue = edges
        triangluation(currentValue, matrixOfValues, points, pointsMock)
    currentValue = end
    triangluation(currentValue, matrixOfValues, points, pointsMock)
    x, y = zip(*polygon)
    plt.plot(x, y)
    path = dijkstra(matrixOfValues, 0, len(points)+1)
    realPath = []
    for i in range(0, len(path)):
        realPath.append(pointsMock[path[i]])
    x, y = zip(*realPath)
    plt.plot(x, y, 'r-')
    plt.show()


if __name__ == '__main__':
    points = [(1, 2), (3, 1), (3, 4), (4, 2), (7, 3), (8, 5), (8, 1), (10, 2), (11, 7), (7, 8), (7, 4), (6, 6), (5, 4),
              (4, 6), (2, 6)]
    start = (2, 3)
    end = (9, 2)
    points1 = [(3, 2), (5, 5), (7, 2), (8, 5), (11, 7), (6, 10), (3, 8), (4, 5)]
    start1 = (4, 4)
    end1 = (7, 3)
    points2 = [(1, 1), (3, 2), (3, 5), (5, 5), (4, 2), (6, 1), (9, 5), (10, 2), (13, 1), (18, 4), (19, 6), (19, 8), (16, 11), (12, 11), (9, 10), (5,12), (1,10), (1, 7), (3, 8), (3, 8), (2, 9), (5, 8), (9, 8), (12, 9), (16, 7), (16, 5), (13, 3), (9, 7), (6,3), (6, 6), (4,7), (1,5)]
    start2 = (1, 1)
    end2 = (2, 8)
    print("Choose you option :")
    print(" <1> Insert how many nodes the polygon has and the nodes + start and end values")
    print(" <2-4> Predefined values")
    val=int(input())
    if val==2:
        shortestPath(start,end,points)
    if val==3:
        shortestPath(start1,end1,points1)
    if val==4:
        pointsMock2 = []
        pointsMock2.append(start)
        pointsMock2.append(end)
        shortestPath(start2,end2,points2)
    else:
        nr=input("Number of points:")
        points.clear()
        for i in range(0,int(nr)):
            print("Insert node x and y values:")
            xvalue = int(input())
            yvalue = int(input())
            node= (xvalue, yvalue)
            points.append(node)
            del node
        print("Insert start x and y values:")
        xvalue=int(input())
        yvalue = int(input())
        del start
        start=(xvalue,yvalue)
        print("Insert end x and y values:")
        xvalue = int(input())
        yvalue = int(input())
        del end
        end=(xvalue,yvalue)
        shortestPath(start,end,points)