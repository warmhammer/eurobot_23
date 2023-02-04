import argparse
import cv2
import os
from os import path
from os.path import abspath


this_dir = path.dirname(abspath(__file__))


def configure_arg_parser() -> argparse.ArgumentParser:
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-ci", "--cameraIndex", type=int, default=0,
                            help="Camera index in system. Default: 0")
    arg_parser.add_argument("-fw", "--frameWidth", type=int, required=True,
                            help="Frame width")
    arg_parser.add_argument("-fh", "--frameHeight", type=int, required=True,
                            help="Frame height")
    arg_parser.add_argument("-op", "--outputPath", type=str, default=f'{this_dir}/../images',
                            help=f"Path - where the images will be saved. Default: '{abspath(f'{this_dir}/../images')}'"
                                 "\nThe script creates folder if it doesn't exist, deletes all existing files in it")
    return arg_parser


def configure_video_capturer():
    cap = cv2.VideoCapture(args['cameraIndex'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args['frameWidth'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args['frameHeight'])
    if not cap.isOpened():
        raise ValueError(f"Can not open camera by index {args['cameraIndex']}")
    return cap


def take_images():
    count = 1
    while capturer.isOpened():
        ret_val, frame = capturer.read()

        if ret_val:
            cv2.imshow('img', frame)
            key = cv2.waitKey(1)

            # save frame to file when pressing spacebar or 's' key
            if key == ord(' ') or key == ord('s'):
                file_path = f"{args['outputPath']}/img{count}.png"
                cv2.imwrite(file_path, frame)
                count += 1
            # exit when pressing 'q' or Esc
            elif key == ord('q') or key == 27:
                break


if __name__ == "__main__":
    ap = configure_arg_parser()
    args = vars(ap.parse_args())
    if not path.isdir(args['outputPath']):
        os.makedirs(args['outputPath'])
    else:
        for f in os.listdir(args['outputPath']):
            os.remove(os.path.join(args['outputPath'], f))

    capturer = configure_video_capturer()
    take_images()
    print(f"Taken images were saved in {abspath(args['outputPath'])}")
    cv2.destroyAllWindows()
    capturer.release()
