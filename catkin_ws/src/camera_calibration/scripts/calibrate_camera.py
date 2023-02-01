import os
import cv2
import glob
import numpy
import sys
from os import path
from os.path import abspath


this_dir = path.dirname(abspath(__file__))

params = {'chessboardCornersCountX': 9, 'chessboardCornersCountY': 6,
          'sizeOfSquareSide': 24, 'frameWidth': 640, 'frameHeight': 480, 'imagesPath': f'{this_dir}/../images'}

if __name__ == "__main__":
    if len(sys.argv) > 1:
        args = sys.argv[1:]

        for i in params.keys():
            if args.__contains__(i) and args.index(i) + 1 < len(args) and args[args.index(i) + 1].isdigit():
                params[i] = int(args[args.index(i) + 1])


################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

    chessboardSize = (params['chessboardCornersCountX'], params['chessboardCornersCountY'])
    frameSize = (params['frameWidth'], params['frameHeight'])

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((chessboardSize[0] * chessboardSize[1], 3), numpy.float32)
    objp[:, :2] = numpy.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

    size_of_chessboard_squares_mm = params['sizeOfSquareSide']
    objp = objp * size_of_chessboard_squares_mm


    # Arrays to store object points and image points from all the images.
    objPoints = []  # 3d point in real world space
    imgPoints = []  # 2d points in image plane.



    if not path.exists(params['imagesPath']):
        print(f"path to images '{params['imagesPath']}' does'nt exists")
        exit(-1)

    images = glob.glob(f"{params['imagesPath']}/*.png")
    if len(images) < 1:
        print(f"there is no .png files in {params['imagesPath']}")
        exit(-1)

    for image in images:

        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objPoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgPoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(300)

    cv2.destroyAllWindows()


############## CALIBRATION #######################################################

    ret, camMtx, dist, rVectors, tVectors = cv2.calibrateCamera(objPoints, imgPoints, frameSize, None, None)

    if ret:

        dir = f'{this_dir}/../camera_info'
        if not path.exists(dir):
            os.makedirs(dir)
        else:
            for f in os.listdir(dir):
                os.remove(os.path.join(dir, f))

        numpy.savez(f'{dir}/camera_info.npz', cameraMatrix=camMtx, distCoeff=dist, rVectors=rVectors, tVectors=tVectors)

        print('calibration was successful\nresult:')
        print('intrinsicMatrix:')
        print(camMtx)
        print('\ndist:')
        print(dist)
        print('\nrVectors:')
        print(rVectors)
        print('\ntVectors:')
        print(tVectors)
    else:
        print('calibration failed')
