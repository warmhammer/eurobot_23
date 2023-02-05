import argparse
import os
import traceback
import cv2
import glob
import numpy as np
from os import path
from os.path import abspath

THIS_DIR = path.dirname(abspath(__file__))


def configure_arg_parser(arg_parser):
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
    arg_parser.add_argument("-ip", "--imagesPath", type=str, default=f'{THIS_DIR}/../images',
                            help="Path to images that will be used for camera calibration. Default path: "
                                 f"'{abspath(f'{THIS_DIR}/../images')}")
    arg_parser.add_argument("-op", "--outputPath", type=str, default=f'{THIS_DIR}/../camera_info',
                            help="Path - where the calibration results will be saved. Default: "
                                 f"'{abspath(f'{THIS_DIR}/../camera_info')}'\nThe script creates folder if it doesn't "
                                 "exist, deletes all existing files in it")


def find_chessboard_corners(chessboard_size, size_of_square_side, images_path):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    objp = objp * size_of_square_side

    # Arrays to store object points and image points from all the images.
    obj_points = []  # 3d point in real world space
    img_points = []  # 2d points in image plane.
    images = glob.glob(f"{images_path}/*.png")
    for image in images:
        img = cv2.imread(image)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret_val, corners = cv2.findChessboardCorners(gray_img, chessboard_size, None)

        # If found, add object points, image points (after refining them)
        if ret_val:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(gray_img, corners, (11, 11), (-1, -1), criteria)
            img_points.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret_val)
            cv2.imshow('img', img)
            cv2.waitKey(300)
    cv2.destroyAllWindows()
    return obj_points, img_points


def save_camera_params_to_file(camera_matrix, dist_coeffs, output_path):
    if not path.isdir(output_path):
        os.makedirs(output_path)
    else:
        for f in os.listdir(output_path):
            os.remove(os.path.join(output_path, f))
    file_path = f'{output_path}/camera_info.npz'
    np.savez(file_path, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    configure_arg_parser(ap)
    args = vars(ap.parse_args())

    imagesPath = abspath(args['imagesPath'])
    outputPath = abspath(args['outputPath'])
    try:
        if not path.isdir(args['imagesPath']):
            raise FileNotFoundError(f"Path to images folder '{imagesPath}' doesn't exists")
            
        if len(glob.glob(f"{imagesPath}/*.png")) < 1:
            raise FileNotFoundError(f"There is no any .png files in '{imagesPath}'")

        frameSize = (args['frameWidth'], args['frameHeight'])

        chessboardSize = (args['chessboardCornersCountX'], args['chessboardCornersCountY'])

        objPoints, imgPoints = find_chessboard_corners(chessboardSize, args['sizeOfSquareSide'], imagesPath)

        if len(objPoints) < 1 or len(imgPoints) < 1:
            raise ValueError(f"Calibration chessboard was not found in images in '{imagesPath}'")

        retVal, camMtx, dist, _, _ = cv2.calibrateCamera(objPoints, imgPoints, frameSize, None, None)

        if retVal:
            save_camera_params_to_file(camMtx, dist, outputPath)

            print('calibration was successful',
                  'camera info:',
                  'cameraMatrix:', camMtx,
                  'distortionCoeffs:', dist,
                  f"results were saved in '{outputPath}/camera_info.npz'",
                  sep='\n\n')
        else:
            print('calibration failed')

    except FileNotFoundError as e:
        print("\033[31m{}\033[0m".format(f'[ ERROR: ] {e}'))

    except ValueError as e:
        print("\033[31m{}\033[0m".format(f'[ ERROR: ] {e}'))

    except cv2.error as e:
        print("\033[31m{}\033[0m".format(f'[ OPENCV ERROR: ] {e}\n{traceback.format_exc()}'))

    except Exception:
        print("\033[31m{}\033[0m".format(traceback.format_exc()))

    finally:
        cv2.destroyAllWindows()
