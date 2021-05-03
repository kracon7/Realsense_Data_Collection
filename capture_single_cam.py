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

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    device_manager = DeviceManager(rs.context(), config)

    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)
    else:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
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
            # get fixed camera image data and save to numpy array
            frames = pipeline.wait_for_frames()
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            
            depth_image = np.asanyarray(aligned_depth_frame.get_data()) * depth_scale
            color_image = np.asanyarray(color_frame.get_data())

            elaps = time.time() - now
            print('Captured image %07d, took %f seconds'%(i, elaps))
            now = time.time()
            
            np.save(os.path.join(save_dir, 'color_%07d.npy'%(i)), color_image)
            np.save(os.path.join(save_dir, 'depth_%07d.npy'%(i)), depth_image)

            # ax[0].imshow(color_image)
            # ax[1].imshow(depth_image)
            # plt.show()

            i += 1
            
    finally:             
        pipeline.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Image collection for extrinsic calibration between fixed camera and manipulator')
    parser.add_argument('--output_dir', default='data', help='directory to store images')
    parser.add_argument('--visualize', default=0, type=int, help='visualize the images or not')
    args = parser.parse_args()
    
    main(args)