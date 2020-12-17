#!/usr/bin/env python
# coding:utf-8

import random, six, sys
import numpy as np

# from chainer import cuda

###PIL version by Shimizu
def random_crop_image(img, size, test=False):
    # _, H, W = img.shape
    W, H = img.size

    ### crop input image with including target bbox
    min_top = 0
    min_left = 0
    w_size = size[0]
    h_size = size[1]
    max_top = H - h_size
    max_left = W - w_size

    if test:
        top = (H - h_size) / 2
        left = (W - w_size) / 2
    else:
        top = random.randint(min_top, max_top)
        left = random.randint(min_left, max_left)

    bottom = top + h_size
    right = left + w_size
    # img = img[:, top:bottom, left:right]
    # img = img[top:bottom, left:right, :]
    return img.crop((left, top, right, bottom))
