import cv2
import numpy as np
import random as rng

def get_rectangular_contours(contours):
    """Approximates provided contours and returns only those which have 4 vertices"""
    res = []
    for contour in contours:
        hull = cv2.convexHull(contour)
        peri = cv2.arcLength(hull, closed=True)
        approx = cv2.approxPolyDP(hull, 0.04 * peri, closed=True)
        if len(approx) == 4:
            res.append(approx)
    return res

img = cv2.imread('./vision/stationPic2.jpg')

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))
src_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# src_gray = cv2.blur(src_gray, (3,3))
cv2.dilate(src_gray, kernel)
cv2.erode(src_gray, kernel)

out = cv2.resize(src_gray, (1280, 720))  # Resize image to fit on screen
cv2.imshow('Canny', out)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Detect edges using Canny
canny_output = cv2.Canny(src_gray, 100, 200)

# Find contours
contours, hierarchy = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# Draw contours
drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
for i in range(len(contours)):
    color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
    cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)

# Show in a window
drawing = cv2.resize(drawing, (1280, 720))  # Resize image to fit on screen
cv2.imshow('Contours', drawing)
cv2.waitKey(0)
cv2.destroyAllWindows()

# print(contours[0])


# Find contours
# contours, hierarchy = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# cntsSorted = sorted(contours, key = cv2.contourArea)
# cntsSorted = cntsSorted[len(cntsSorted)-10:]

cntsSorted = get_rectangular_contours(contours)

# Draw contours
drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
# drawing = img
for i in range(len(cntsSorted)):
    color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
    cv2.drawContours(drawing, cntsSorted, i, color, 2, cv2.LINE_8, hierarchy, 0)
    # cv2.fillPoly(drawing, pts=cntsSorted[i], color=color)

# Show in a window
drawing = cv2.resize(drawing, (1280, 720))  # Resize image to fit on screen
# img = cv2.resize(img, (1920, 1080))  # Resize image to fit on screen
cv2.imshow('Contours', drawing)
cv2.waitKey(0)
cv2.destroyAllWindows()