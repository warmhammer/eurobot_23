import cv2
import sys
import os
from os import path
from os.path import abspath


params = {'cameraIndex': 0, 'frameWidth': 640, 'frameHeight': 480}

if __name__ == "__main__":
    if len(sys.argv) > 1:
        args = sys.argv[1:]

        for i in params.keys():
            if args.__contains__(i) and args.index(i) + 1 < len(args) and args[args.index(i) + 1].isdigit():
                params[i] = int(args[args.index(i) + 1])

    cap = cv2.VideoCapture(params['cameraIndex'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, params['frameWidth'])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, params['frameHeight'])

    if not cap.isOpened():
        print('Can not open camera by index')
        exit(0)


    this_dir = path.dirname(abspath(__file__))
    dir = f'{this_dir}/../images'
    if not path.exists(dir):
        os.makedirs(dir)
    else:
        for f in os.listdir(dir):
            os.remove(os.path.join(dir, f))


    count = 1
    while cap.isOpened():
        ret, frame = cap.read()

        if ret:
            cv2.imshow('img', frame)
            key = cv2.waitKey(1)

            # save frame to file when pressing spacebar or 's' key
            if key == ord(' ') or key == ord('s'):

                

                cv2.imwrite(f"{this_dir}/../images/img{count}.png", frame)
                count += 1

            elif key == ord('q') or key == 27:
                break

    cv2.destroyAllWindows()
    cap.release()
