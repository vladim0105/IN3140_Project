# from cv2 import cv2

import cv2
import numpy as np


def imageToPath(imageFile, liftHeight, isCareful):
    # Load in image as grayscale (so that we only work with one color channel)
    img = cv2.imread(imageFile)
    edges = cv2.Canny(img, 100, 250)
    positions = positionsFromEdges(edges)
    print(len(positions))
    # Optimize x-lines. Positions already sorted by y, then x
    optimizedX, unoptimized = optimizeX(positions)
    # Sort unoptimzed points by x, then y in order the get the y-lines
    unoptimized = np.array(sorted(unoptimized, key=lambda item: (item[0], item[1])))
    # Optimize y-lines
    optimizedY, unoptimized = optimizeY(unoptimized)

    totalPath = combine(optimizedX, optimizedY, unoptimized)
    finalPath = finalizePath(totalPath, liftHeight, isCareful)
    np.savetxt("path_debug.txt", finalPath, fmt="%1.1i")

    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("output", 500, 500)
    cv2.imshow("output", edges)
    cv2.waitKey(0)

    return


def finalizePath(path, liftHeight, proper):
    finalPath = []
    for entry in path:
        newEntry = np.array([entry[0], entry[1], entry[2]])
        liftEntry = np.array([entry[0], entry[1], liftHeight])
        if entry[3] <= 0:
            finalPath.append(liftEntry)
        finalPath.append(newEntry)
        if entry[3] > 0 and proper:
            finalPath.append(liftEntry)

    return np.array(finalPath)


def combine(*args):
    finalPath = []
    for arg in args:
        for el in arg:
            finalPath.append(el)

    return np.array(finalPath)


def positionsFromEdges(edges):
    y, x = np.where(edges == 255)
    positions = []
    for posx, posy in zip(x, y):
        # x,y,z,pathlength
        positions.append(np.array([posx, posy, 0, 0]))
    return np.array(positions)


def optimizeX(positions):
    path = []
    xdir = False
    length = 0
    for i in range(1, len(positions)):
        prev = positions[i - 1]
        current = positions[i]
        diff = np.subtract(current, prev)
        if np.array_equal(diff, np.array([1, 0, 0, 0])):
            if xdir == False:
                prev[3] = -1
                path.append(prev)
            xdir = True
            length += 1
            continue
        prev[3] = length
        path.append(prev)
        xdir = False
        length = 0
    positions[len(positions) - 1][3] = length
    path.append(positions[len(positions) - 1])
    return splitOptimizedFromUnoptimized(path)


def optimizeY(positions):
    path = []
    ydir = False
    length = 0
    for i in range(1, len(positions)):
        prev = positions[i - 1]
        current = positions[i]
        diff = np.subtract(current, prev)
        if np.array_equal(diff, np.array([0, 1, 0, 0])):
            if ydir == False:
                prev[3] = -1
                path.append(prev)
            ydir = True
            length += 1
            continue
        prev[3] = length
        path.append(prev)
        ydir = False
        length = 0
    positions[len(positions) - 1][3] = length
    path.append(positions[len(positions) - 1])
    return splitOptimizedFromUnoptimized(path)


def splitOptimizedFromUnoptimized(path):
    optimized = []
    unoptimized = []
    for position in path:
        if position[3] == 0:
            unoptimized.append(position)
        else:
            optimized.append(position)
    return np.array(optimized), np.array(unoptimized)
