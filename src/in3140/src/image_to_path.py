from cv2 import cv2
import numpy as np


def getContourPaths(imageFile):
    # Load in image as grayscale (so that we only work with one color channel)
    img = cv2.imread(imageFile)
    edges = cv2.Canny(img, 100, 250)

    contours = findContours(edges)

    img_contours = np.zeros(img.shape)
    for i in range(len(contours)):
        color = (np.random.rand() % 255, np.random.rand() % 255, np.random.rand() % 255)
        cv2.drawContours(img_contours, contours, i, color, 1)

    cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("image", 500, 500)
    cv2.imshow("image", img_contours)
    cv2.waitKey(0)

    return contours


def findContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # create an empty image for contours

    # draw the contours on the empty image

    return contours


# getContourPaths("pepe.jpg")
