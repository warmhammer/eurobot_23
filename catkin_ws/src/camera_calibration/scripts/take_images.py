import argparse
import cv2
import os
import traceback
from os import path
from os.path import abspath

THIS_DIR = path.dirname(abspath(__file__))


def configure_arg_parser(arg_parser):
    arg_parser.add_argument("-ci", "--cameraIndex", type=int, default=0,
                            help="Camera index in system. Default: 0")
    arg_parser.add_argument("-fw", "--frameWidth", type=int, required=True,
                            help="Frame width")
    arg_parser.add_argument("-fh", "--frameHeight", type=int, required=True,
                            help="Frame height")
    arg_parser.add_argument("-op", "--outputPath", type=str, default=f'{THIS_DIR}/../images',
                            help=f"Path - where the images will be saved. Default: '{abspath(f'{THIS_DIR}/../images')}'"
                                 "\nThe script creates folder if it doesn't exist, deletes all existing files in it")


def configure_video_capturer(camera_index, frame_width, frame_height):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    if not cap.isOpened():
        raise ValueError(f"Can not open camera by index {args['cameraIndex']}")

    return cap


def take_images(video_capturer, output_path):
    count = 1
    while video_capturer.isOpened():
        ret_val, frame = video_capturer.read()

        if ret_val:
            cv2.imshow('img', frame)
            key = cv2.waitKey(1)

            # save frame to file when pressing spacebar or 's' key
            if key == ord(' ') or key == ord('s'):
                file_path = f"{output_path}/img{count}.png"
                cv2.imwrite(file_path, frame)
                count += 1
            # exit when pressing 'q' or Esc
            elif key == ord('q') or key == 27:
                break


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    configure_arg_parser(ap)
    args = vars(ap.parse_args())
    outputPath = abspath(args['outputPath'])
    try:
        if not path.isdir(outputPath):
            os.makedirs(outputPath)
        else:
            for f in os.listdir(outputPath):
                os.remove(os.path.join(outputPath, f))

        try:
            capturer = configure_video_capturer(args['cameraIndex'], args['frameWidth'], args['frameHeight'])
            take_images(capturer, outputPath)
            capturer.release()

        except ValueError as e:
            print("\033[31m{}\033[0m".format(f'[ ERROR: ] {e}'))
            raise

        except Exception:
            raise

        else:
            print(f"Taken images were saved in '{outputPath}'")

        finally:
            cv2.destroyAllWindows()

    except cv2.error as e:
        print("\033[31m{}\033[0m".format(f'[ OPENCV ERROR: ] {e}\n{traceback.format_exc()}'))

    except Exception:
        print("\033[31m{}\033[0m".format(traceback.format_exc()))

    finally:
        pass
