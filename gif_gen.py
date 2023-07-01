# making a video from imgs in img_pairs/match_seq
import cv2
import os
import glob
import numpy as np
import imageio

def make_video():
    img_array = []

    for filename in glob.glob('img_pairs/match_seq/*.png'):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)

    out = cv2.VideoWriter('img_pairs/match_seq.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 3, size)

    for file in sorted(glob.glob('img_pairs/match_seq/*.png')):
        img = cv2.imread(file)
        out.write(img)
    out.release()


def generate_gif(img_dir, gif_name):
    images = []
    count = 0
    for file_name in sorted(os.listdir(img_dir)):
        if file_name.endswith('.png'):
            file_path = os.path.join(img_dir, file_name)
            #crop to central 
            img = imageio.imread(file_path)
            images.append(img)
        count += 1
    #save the gif, looping indefinitely
    imageio.mimsave(gif_name, images, duration=500.0,loop=0)
    print(f'Gif saved to {gif_name}')

if __name__ == '__main__':
    generate_gif('img_pairs/match_seq', gif_name='img_pairs/match_seq/match_seq.gif')