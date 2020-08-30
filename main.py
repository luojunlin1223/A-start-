from __future__ import print_function
import cv2
import os
import numpy as np
import math
import heapq as hq

path = "Charge.jpg"


class Array2D:
    """
        Description: This is a class for constructing maps.
            1. The construction method requires two parameters, namely the width and height of the two-dimensional array
            2. The default value of the array is 0
    """

    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.data = []
        self.data = [[0 for y in range(h)] for x in range(w)]

    def __getitem__(self, item):
        return self.data[item]


class Point:
    """
    Description: This is a class for constructing coordinate points.
         1. Coordinate points can be obtained directly
         2.eq function for judging equal
         3.str is used to represent
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def __str__(self):
        return "x:" + str(self.x) + ",y:" + str(self.y)


class AStar:
    """
    Description: This is a class for building A* algorithms.
        1. Includes a Node class to construct a node
        2. Design methods for heuristic functions.
          Calculate the Color difference using RGB separately.
          Using thresholds to judge the generation of hindrances
          The H-value of travel in an obstacle consists of two parts.
          color&distance(each assigned a weight value representing the relevant proportion;
          assigned a size representing
   
    """

    class Node:  # Describe the node data in the ASTAR algorithm
        def __lt__(self, other):
            return self.g + self.h < other.g + other.h

        def __init__(self, point, endPoint, g=0):
            self.point = point
            self.father = None
            self.g = g
            self.h = self.heuristicFun(point, endPoint)

        def heuristicFun(self, point, endPoint):
            Red = abs(int(im[point.y, point.x][0] - int(im[endPoint.y, endPoint.x][0])))
            Green = abs(
                int(im[point.y, point.x][1] - int(im[endPoint.y, endPoint.x][1]))
            )
            Blue = abs(
                int(im[point.y, point.x][2] - int(im[endPoint.y, endPoint.x][2]))
            )
            color = Red + Green + Blue
            distance = abs(point.x - endPoint.x) + abs(point.y - endPoint.y)
            if color >= 300:
                h = distance * 50 + color * 80
            else:
                h = distance * 10
            return h

    def __init__(self, map2d, startPoint, endPoint, passTag=0):

        # openList
        self.openList = []
        # closeList
        self.closeList = []
        # Map
        self.map2d = map2d
        # endpoint
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)

        # Tag
        self.passTag = passTag

    def getMinNode(self):
        """
        The node with the smallest F value in the openList
        
        Since we're looking for the element with the smallest F value, we'll put it at the top of the heap.
        This element has two child nodes, each with an F value equal to, or slightly higher than, the element
        Each child node has two more child nodes, which in turn have child nodes that are equal to them or slightly higher
        An interesting fact about binaries is that you can simply store it in a one-dimensional array
        In this array, the element at the top of the heap should be the first element of the array (it's a subscript 1, not a 0).
        Two child nodes will be in the positions of 2 and 3. The 4 child nodes of these two nodes should be in the 4-7 position.
        Two child nodes of any element can be obtained by multiplying the position of the current element by 2 (to get the first child node) and by multiplying 2 plus 1 (to get the second child node)
        At the same time, new elements are added in left-to-right order
        Add elements.
        Roughly, in order to add an element to the heap, we put it at the end of the array
        Then compared to its parent at the current position/2, the fraction is rounded
        If the new element has a lower F value, we swap the two elements
        We repeat this process until the element is no longer lower than its parent, or the element has reached the top and is at position 1 of the array.
        10 30 20 34 38 20 24 17
        10 30 20 17 38 20 24 34
        10 17 20 30 38 30 24 34
        
        Delete: First, we delete the element in position 1, which is now empty. We then take the last element of the pile and move to position 1.
        34 17 20 30 38 30 24
        We then compare it with two child nodes that are at position (current position * 2) and (current position * 2 + 1), respectively. If it is lower than the F value of both child nodes, it stays in place.
        Instead, swap it with a lower child node.
        17 34 20 30 38 20 24
        17 30 20 34 38 30 24
        
        :return: Node
        """
        currentNode = hq.heappop(self.openList)
        return currentNode

    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInopenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY):

        # Cross-border detection
        if (
            minF.point.x + offsetX < 0
            or minF.point.x + offsetX > self.map2d.w - 1
            or minF.point.y + offsetY < 0
            or minF.point.y + offsetY > self.map2d.h - 1
        ):
            return
        # If it's an obstacle, ignore it.
        if self.map2d[minF.point.x + offsetX][minF.point.y + offsetY] != self.passTag:
            return
        # If in the closing table, ignore the
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
        if self.pointInCloseList(currentPoint):
            return
        # Set unit cost
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14
        # If no longer in openList, add it to openList
        currentNode = self.pointInopenList(currentPoint)
        if not currentNode:
            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
            currentNode.father = minF
            hq.heappush(self.openList, currentNode)
            return
        # If in openList, determine if the G of minF to the current point is smaller
        if (
            minF.g + step < currentNode.g
        ):  # If it's smaller, recalculate the g value and change the FATHER
            currentNode.g = minF.g + step
            currentNode.father = minF

    def start(self):

        if self.map2d[self.endPoint.x][self.endPoint.y] != self.passTag:
            return None

        # 1. Put the starting point in the open list
        startNode = AStar.Node(self.startPoint, self.endPoint)
        hq.heappush(self.openList, startNode)
        # 2. Master loop logic
        while True:
            # Find the point with the lowest F value
            minF = self.getMinNode()
            # Add this point to the closeList and remove it from the openList
            self.closeList.append(minF)
            # Judging this node up and down and left and right nodes
            self.searchNear(minF, 0, -1)
            self.searchNear(minF, 0, 1)
            self.searchNear(minF, -1, 0)
            self.searchNear(minF, 1, 0)
            self.searchNear(minF, 1, -1)
            self.searchNear(minF, 1, 1)
            self.searchNear(minF, -1, 1)
            self.searchNear(minF, 1, 1)
            # To determine whether to terminate
            point = self.endPointInCloseList()
            if len(self.closeList) > 3000:
                cPoint = self.closeList[-1]
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        return list(reversed(pathList)), False
            if point:  # If the endpoint is in the closing table, return the result

                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        return list(reversed(pathList)), True
            if len(self.openList) == 0:
                return None


if __name__ == "__main__":

    mychoice = []

    def read_img(path):
        """Given a path to an image file, returns a cv2 array
        str -> np.ndarray"""
        if os.path.isfile(path):
            return cv2.imread(path)
        else:
            raise ValueError("There are something wrong in the path!")

    def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            cv2.circle(im, (x, y), 1, (255, 0, 0), thickness=-1)
            cv2.imshow("image", im)
            mychoice.append(x)
            mychoice.append(y)

    im = read_img(path)
    cv2.namedWindow("image", 0)
    cv2.resizeWindow("image", 1024, 648)
    cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    cv2.imshow("image", im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    h = im.shape[0]
    w = im.shape[1]

    array2d = Array2D(w, h)
    astart = AStar(
        array2d, Point(mychoice[0], mychoice[1]), Point(mychoice[2], mychoice[3])
    )
    trace, check = astart.start()
    while check == False:
        astart = AStar(
            array2d,
            Point(astart.closeList[-1].point.x, astart.closeList[-1].point.y),
            Point(mychoice[2], mychoice[3]),
        )
        Newtrace, check = astart.start()
        trace = trace + Newtrace
    for i in range(0, len(trace) - 1):
        cv2.line(
            im,
            (trace[i].x, trace[i].y),
            (trace[i + 1].x, trace[i + 1].y),
            (0, 255, 0),
            1,
            8,
            0,
        )
    cv2.namedWindow("Final", 0)
    cv2.resizeWindow("Final", 1024, 648)
    cv2.imshow("Final", im)
    cv2.waitKey(0)
