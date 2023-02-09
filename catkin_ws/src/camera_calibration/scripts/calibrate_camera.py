import argparse
import cv2
import glob
import numpy as np
import os
from os import path
from os.path import abspath
import textwrap
import traceback


def parse_arguments():
    this_dir = path.dirname(abspath(__file__))
    arg_parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                         description=textwrap.dedent('''\
        ================================== Description ===================================
        This script performs  camera  calibration  based on  pictures  of  the calibration
        board taken by the camera whose parameters you want to  define. It  is recommended
        to use at least twenty pictures for calibration. Camera  parameters will be  saved
        in camera_info.npz file. Below is the python code by which  you can get the camera
        parameters from this file:

        with numpy.load(camera_info_path) as file:
            camera_matrix, dist_coeffs = [file[i] for i in ('cameraMatrix', 'distCoeffs')]

        For more information about camera calibration:
        https://learnopencv.com/camera-calibration-using-opencv/
        =================================================================================='''))

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

    return vars(arg_parser.parse_args())


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


def calibrate_camera(frame_size, chessboard_size, size_of_square_side, images_path, output_path):
    if not path.isdir(images_path):
        raise FileNotFoundError(f"Path to images folder '{images_path}' doesn't exists")

    if len(glob.glob(f"{images_path}/*.png")) < 1:
        raise FileNotFoundError(f"There is no any .png files in '{images_path}'")

    obj_points, img_points = find_chessboard_corners(chessboard_size, size_of_square_side, images_path)

    if len(obj_points) < 1 or len(img_points) < 1:
        raise ValueError(f"Calibration chessboard was not found in images in '{images_path}'")

    ret_val, cam_mtx, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, frame_size, None, None)

    if ret_val:
        save_camera_params_to_file(cam_mtx, dist, output_path)

        print('calibration was successful',
              'camera info:',
              'cameraMatrix:', cam_mtx,
              'distortionCoeffs:', dist,
              f"results were saved in '{output_path}/camera_info.npz'",
              sep='\n\n')
    else:
        print('calibration failed')


if __name__ == "__main__":
    args = parse_arguments()

    frameSize = (args['frameWidth'], args['frameHeight'])
    chessboardSize = (args['chessboardCornersCountX'], args['chessboardCornersCountY'])
    sizeOfSquareSide = args['sizeOfSquareSide']
    imagesPath = abspath(args['imagesPath'])
    outputPath = abspath(args['outputPath'])
    try:
        calibrate_camera(frameSize, chessboardSize, sizeOfSquareSide, imagesPath, outputPath)

    except (FileNotFoundError, ValueError) as e:
        print("\033[31m{}\033[0m".format(f'[ ERROR: ] {e}'))

    except cv2.error as e:
        print("\033[31m{}\033[0m".format(f'[ OPENCV ERROR: ] {e}\n{traceback.format_exc()}'))

    except Exception:
        print("\033[31m{}\033[0m".format(traceback.format_exc()))

    finally:
        cv2.destroyAllWindows()
