#!/usr/bin/env python3
from pathlib import Path
import argparse
from oaklib import *


def main(args):
    imgs, imu_data = grab_oak(args["num"])
    
    print(f"Camera[{len(imgs)}]: {len(imgs)/(imgs[-1][-1] - imgs[0][-1]):.1f} hz end time {imgs[-1][-1]}")
    print(f"IMU[{len(imu_data)}]: {len(imu_data)/(imu_data[-1][-1] - imu_data[0][-1]):.1f} hz end time {imu_data[-1][-1]}")
    
    # save images
    folder = args["path"]

    # save_camera_params(f"{folder}/cal.yml")

    fmt = args["format"].lower()
    if fmt not in ['jpg', 'png']:
        fmt = 'jpg'

    write_images(imgs, folder, fmt)
    if args["video"]: write_video([img for img,_ in imgs[1:]], folder + "/video.mp4")
    
    # save IMU
    col_names = ["ax","ay","az","gx","gy","gz","timestamp"]
    write_csv(folder + "/imu.csv", imu_data, col_names)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=f'A simple \
    program to capture images from a camera.')

    parser.add_argument('-p', '--path', help='name of folder to save data to, default: camera', default='camera')
    parser.add_argument('-n', '--num', type=int, help='number of images to capture, default is 0', default=50)
    parser.add_argument('-f', '--format', help="save image format as jpg or png, default is jpg", default="jpg")
    parser.add_argument('-v', '--video', action='store_true', help="save images as video, default is False", default=False)

    args = vars(parser.parse_args())

    print(f"Capturing {args['num']} images as {args['format']}")
    print(f"Runtime: {args['num']/30:.1f} sec")
    print(f"Location: {Path(args['path'])}")
    
    main(args)