import argparse
import os
import cv2
import glob
import numpy
from os import path
from os.path import abspath

this_dir = path.dirname(abspath(__file__))


def configure_arg_parser():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-cx", "--chessboardCornersCountX", type=int, required=True,
                            help="Number of internal corners of the calibration chessboard horizontally")
    arg_parser.add_argument("-cy", "--chessboardCornersCountY", type=int, required=True,
                            help="Number of internal corners of the calibration chessboard vertically")
    arg_parser.add_argument("-ss", "--sizeOfSquareSide", type=int, required=True,
                            help="size of the side of the square of the calibration chessboard in mm")
    arg_parser.add_argument("-fw", "--frameWidth", type=int, required=True,
                            help="Frame width of the images")
    arg_parser.add_argument("-fh", "--frameHeight", type=int, required=True,
                            help="Frame height of the images")
    arg_parser.add_argument("-ip", "--imagesPath", type=str, default=f'{this_dir}/../images',
                            help="Path to images that will be used for camera calibration. Default path: "
                                 f"'{abspath(f'{this_dir}/../images')}")
    arg_parser.add_argument("-op", "--outputPath", type=str, default=f'{this_dir}/../camera_info',
                            help="Path - where the calibration results will be saved. Default: "
                                 f"'{abspath(f'{this_dir}/../camera_info')}'\nThe script creates folder if it doesn't "
                                 "exist, deletes all existing files in it")
    return arg_parser


def find_chessboard_corners():
    chessboard_size = (args['chessboardCornersCountX'], args['chessboardCornersCountY'])

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = numpy.zeros((chessboard_size[0] * chessboard_size[1], 3), numpy.float32)
    objp[:, :2] = numpy.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    size_of_chessboard_squares_mm = args['sizeOfSquareSide']
    objp = objp * size_of_chessboard_squares_mm

    # Arrays to store object points and image points from all the images.
    obj_points = []  # 3d point in real world space
    img_points = []  # 2d points in image plane.
    images = glob.glob(f"{args['imagesPath']}/*.png")
    if len(images) < 1:
        raise FileNotFoundError(f"There is no any .png files in '{path.abspath(args['imagesPath'])}'")
    for image in images:

        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret_val, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        # If found, add object points, image points (after refining them)
        if ret_val:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img_points.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret_val)
            cv2.imshow('img', img)
            cv2.waitKey(300)
    cv2.destroyAllWindows()
    return obj_points, img_points


def save_to_file():
    if not path.isdir(args['outputPath']):
        os.makedirs(args['outputPath'])
    else:
        for f in os.listdir(args['outputPath']):
            os.remove(os.path.join(args['outputPath'], f))
    file_path = f"{args['outputPath']}/camera_info.npz"
    numpy.savez(file_path, cameraMatrix=camMtx, distCoeffs=dist)


if __name__ == "__main__":
    ap = configure_arg_parser()
    args = vars(ap.parse_args())
    if not path.isdir(args['imagesPath']):
        raise FileNotFoundError(f"Path to images '{abspath(args['imagesPath'])}' doesn't exists")

    frameSize = (args['frameWidth'], args['frameHeight'])

    objPoints, imgPoints = find_chessboard_corners()

    retVal, camMtx, dist, _, _ = cv2.calibrateCamera(objPoints, imgPoints, frameSize, None, None)

    if retVal:
        save_to_file()

        print('calibration was successful',
              'camera info:',
              'cameraMatrix:', camMtx,
              'distortionCoeffs:', dist,
              f'results were saved in {abspath(args["outputPath"])}/camera_info.npz',
              sep='\n\n')
    else:
        print('calibration failed')
