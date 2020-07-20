import cv2
import numpy as np

# assuming background pixels < line pixels

def offset(img, x, y, value_range):
    M = np.float32([[1,0,x], [0,1,y]])
    r_img = cv2.warpAffine(img, M, img.shape)
    r_img[np.where(r_img<value_range[0])] = value_range[0]
    r_img[np.where(r_img>value_range[1])] = value_range[1]

    return r_img

def check_range(begin, end, fixed_size):
    # for canceld digits in resizing shapes
    if (end - begin) > fixed_size:
        return begin, end-1
    elif (end - begin) < fixed_size:
        return begin, end+1
    else:
        return begin, end

def scal(img, ratio, value_range):
    orig_rows, orig_cols = img.shape
    scaled = cv2.resize(img, None, fx=ratio, fy=ratio, interpolation = cv2.INTER_CUBIC)
    rows, cols = scaled.shape
    if ratio > 1:
        rows_begin, rows_end = check_range(rows/2-orig_rows/2, rows/2+orig_rows/2, orig_rows)
        cols_begin, cols_end = check_range(cols/2-orig_cols/2, cols/2+orig_cols/2, orig_cols)
        r_img = scaled[rows_begin:rows_end, cols_begin:cols_end]
    else:
        r_img = np.ones(img.shape) * value_range[0]
        rows_begin, rows_end = check_range(orig_rows/2-rows/2, orig_rows/2+rows/2, rows)
        cols_begin, cols_end = check_range(orig_cols/2-cols/2, orig_cols/2+cols/2, cols)
        r_img[rows_begin:rows_end, cols_begin:cols_end] = scaled

    r_img[np.where(r_img<value_range[0])] = value_range[0]
    r_img[np.where(r_img>value_range[1])] = value_range[1]
    return r_img
    
def rotate(img, angle, value_range):
    rows, cols = img.shape
    #check = np.ones(img.shape)
    M = cv2.getRotationMatrix2D((cols/2, rows/2), angle, 1)
    r_img = cv2.warpAffine(img, M, img.shape)    
    #check = cv2.warpAffine(check, M, img.shape)
    #r_img[np.where(check==0.0)] = value_range[0]

    r_img[np.where(r_img<value_range[0])] = value_range[0]
    r_img[np.where(r_img>value_range[1])] = value_range[1]

    return r_img
