import numpy as np
import cv2 as cv
import glob
import matplotlib.pyplot as plt
import json
import os

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:8].T.reshape(-1, 2)
objp = objp * 28
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('calibrate_imgs/*[0-9].png')
test_img = 'test.png'
mtx_file = 'in_mtx.npy'
dist_file = 'dist.npy'

if os.path.exists(mtx_file) and os.path.exists(dist_file):
    print("find param file")

    with open(mtx_file, 'rb') as f:
        mtx = np.load(mtx_file)
        print(f"mtx:\n{mtx}")
    with open(dist_file, 'rb') as f:
        dist = np.load(dist_file)
        print(f"dist:\n{dist}")

else: 

    print(f"find {len(images)} images")

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (6, 8), None)
        
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            # cv.drawChessboardCorners(img, (8, 6), corners2, ret)
            # cv.imshow('img', img)
            # cv.waitKey(1000)
        else: print(f"Not found pattern in {fname}.")

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # print(ret, mtx, dist, rvecs, tvecs)

    with open('in_mtx.npy', 'wb') as f:
        np.save(f, mtx)
    with open('dist.npy', 'wb') as f:
        np.save(f, dist)

plt.figure(figsize=(10, 8))

for i, fname in enumerate(images):

    img = cv.imread(fname)
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    h,  w = img.shape[:2]
    # print(h, w)
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    # print(f"new camera matrix: \n{newcameramtx}")
    # undistort
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    plt.subplot(4, 5, i+1)
    plt.imshow(dst)

plt.tight_layout()
plt.savefig('undistorted.png')
plt.figure(figsize=(10, 8))
test = cv.imread(test_img)
test = cv.cvtColor(test, cv.COLOR_RGB2BGR)
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
dst = cv.undistort(test, mtx, dist, None, newcameramtx)
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
plt.imshow(dst)
plt.savefig('test_output.png')
