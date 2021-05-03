import os
import sys
import argparse
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
import datetime as dt
import time

from realsense_device_manager import DeviceManager

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def save_jpg(array, path):
    # creating image object of
    # above array
    data = im.fromarray(array)
      
    # saving the final output 
    # as a PNG file
    data.save(path)


def main(args):

    # set up save dir
    t = dt.datetime.now()

    save_dir = os.path.join(ROOT, args.output_dir, '%d-%d_%d-%d'%(t.month, t.day, t.hour, t.minute))
    os.system('mkdir -p {}'.format(save_dir))

    dispose_frames_for_stablisation = 20

    # initialize camera node for fixed camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    device_manager = DeviceManager(rs.context(), config)
    available_devices = device_manager._available_devices

    # create folder for each device
    device_dirs = {}
    for dvc in available_devices:
        device_dirs[dvc] = os.path.join(save_dir, dvc)
        os.system('mkdir -p {}'.format(device_dirs[dvc]))

    device_manager.enable_all_devices()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = device_manager._enabled_devices[available_devices[0]].pipeline_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    align_to = rs.stream.color
    align = rs.align(align_to)

    if args.visualize:
        fig, ax = plt.subplots(2,1)

    for frame in range(dispose_frames_for_stablisation):
        device_manager.poll_frames()

    i = 0
    now = time.time()

    # Streaming loop
    try:
        while True:
            
            now = time.time()

            aligned_frames = {}
            for dvc in available_devices:
                device_manager._enabled_devices[dvc].pipeline_profile.get_streams()
                frames = device_manager._enabled_devices[dvc].pipeline.wait_for_frames()
                # Align the depth frame to color frame
                aligned_frames[dvc] = align.process(frames)

            print('Captured frame %07d, took %f seconds'%(i, time.time()-now))

            now = time.time()        

            # Get aligned frames
            for dvc in aligned_frames.keys():
                depth_frame = aligned_frames[dvc].get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames[dvc].get_color_frame()
            
                depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale
                color_image = np.asanyarray(color_frame.get_data())

                np.save(os.path.join(device_dirs[dvc], 'color_%07d.npy'%(i)), color_image)
                np.save(os.path.join(device_dirs[dvc], 'depth_%07d.npy'%(i)), depth_image)

            print('Saved frame, took %f seconds'%(time.time()-now))
            
            
            # ax[0].imshow(color_image)
            # ax[1].imshow(depth_image)
            # plt.show()

            i += 1
            
    finally:             
        device_manager.disable_streams()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Image collection for extrinsic calibration between fixed camera and manipulator')
    parser.add_argument('--output_dir', default='data', help='directory to store images')
    parser.add_argument('--visualize', default=0, type=int, help='visualize the images or not')
    args = parser.parse_args()
    
    main(args)