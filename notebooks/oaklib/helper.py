import cv2
import numpy as np  # matrix manipulations
import csv
import platform
from pathlib import Path
import zipfile
import os
import re
import zipfile
import shutil

def write_csv(filename, data, col_names=None):
    """
    Writes data to CSV file
    """
    with open(filename, 'w', newline='') as fd:
        writer = csv.writer(fd)
        if col_names: writer.writerow(col_names)
        for d in data:
            writer.writerow(d)

def read_csv(filename):
    data = None
    with open(filename, 'r') as fd:
        reader = csv.reader(fd)
        data = [row for row in reader]

    num = False
    for d in data[0]:
        if isinstance(d, float):
            num = True
        else:
            break
    if num is False:
        data = data[1:]
        
    return np.array(data)

def write_video(frames, filename, fps=30, fmt=None):
    frame_height, frame_width = frames[0].shape[:2]
    
    # the encode hates grayscale, convert to RGB
    if len(frames[0].shape) == 2:
        convert = True
    else:
        convert = False
    
    # pick a good encoder for the current OS
    sys = platform.system()
    if fmt is None:
        if sys in ['Darwin']:
            fmt = 'avc1'
        else:
            fmt = 'mjpg'

    fourcc = cv2.VideoWriter_fourcc(*fmt)
        
    out = cv2.VideoWriter(
        filename,
        fourcc, 
        fps, 
        (frame_width,frame_height))
    
    for frame in frames:
        if convert:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        out.write(frame)
    
    out.release()

def read_video(filename, split=False):
    """
    read a video and output an array of images
    
    Args: 
    - filename: path to video
    - split: frame into left/right images, False is default
    Return: [frame, ...] or [(left,right), ...]
    """
    cap = cv2.VideoCapture(filename)
    if not cap.isOpened():
        return None
    imgs = []
    while True:
        ok, frame = cap.read()
        if ok:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            if split:
                h,w = frame.shape
                left = frame[:,:w//2]
                right = frame[:,w//2:]
                imgs.append((left,right))
            else:
                imgs.append(frame)
        else:
            break
    cap.release()
    return imgs

def write_images(frames, path, fmt="png", grayscale=True):
    """
    Writes images to folder

    Args:
    - frames: array of images with timestamp [(frame,timestamp), ...)
    - path: folder location to save images to
    - fmt: format of image (png, jpg, tif), default is png
    - grayscale: save image as grayscale, default is True
    Returns: True successful or False (with printed message) on error
    """
    if not isinstance(path, Path):
        path = Path(path)
        
    frame_height, frame_width = frames[0][0].shape[:2]

    if grayscale:
        if len(frames[0][0].shape) == 2:
            convert = False
        else:
            convert = True

    if fmt[0] == '.':
        fmt = fmt[1:]
    fmt = fmt.lower()

    if fmt not in ["png", "jpg", "tif"]:
        print("Error: invalid image format: {fmt}")
        return False

    if path.exists():
        # Remove the directory and all its contents
        shutil.rmtree(path)
        
    path.mkdir(parents=True, exist_ok=True)

    datum = frames[0][-1]

    for frame, ts in frames:
        if convert:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ts = ts - datum
        filename = path / Path(f"{int(ts*1E6):010d}.{fmt}")
        
        ok = cv2.imwrite(filename, frame)
        if not ok:
            print(f"Error: image could not be saved to {filename}")
            return False
            
    return True

def read_images(folder_path):
    """
    """
    def numeric_sort_key(path):
        # Extract numeric parts from the filename
        return [int(text) if text.isdigit() else text.lower() 
                for text in re.split('([0-9]+)', path.name)]

    # def image_only(path):
    #     return [file for file in path.iterdir() if file.suffix is in ["jpg", "png"]]

    if not isinstance(folder_path, Path):
        folder_path = Path(folder_path)
    
    # Get all files in the folder, excluding directories
    files = [file for file in folder_path.iterdir() if file.is_file() and (file.suffix in [".jpg", ".png"])]
    # print(files)
    
    # Sort the list of files by name
    sorted_files = sorted(files, key=numeric_sort_key)

    imgs = []
    for file in sorted_files:
        ts = float(file.stem) / 1E6
        img = cv2.imread(file, cv2.IMREAD_GRAYSCALE)
        imgs.append((img, ts))

    return imgs

def zip_folder(folder_path, output_filename):
    """
    """
    # Convert string to Path object if necessary
    folder_path = Path(folder_path)
    
    # Check if folder exists
    if not folder_path.exists():
        print(f"The folder {folder_path} does not exist!")
        return False

    # Create a ZipFile object with write mode
    with zipfile.ZipFile(output_filename, 'w', zipfile.ZIP_DEFLATED) as zipf:
        # Walk through all the files in the directory recursively
        for file_path in folder_path.rglob('*'):
            if file_path.is_file() and (file.suffix in [".jpg", ".png", ".csv"]):
                # Use relative_to to maintain folder structure
                relative_path = file_path.relative_to(folder_path)
                # Write the file to the zip archive
                zipf.write(file_path, relative_path)

    return True



            
        
        