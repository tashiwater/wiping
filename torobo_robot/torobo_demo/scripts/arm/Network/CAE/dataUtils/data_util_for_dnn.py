import numpy as np
from IPython.core.debugger import Tracer; keyboard = Tracer()
from PIL import Image
from matplotlib import pylab as plt

def nomalization(data, indataRange, outdataRange):
    data = ( data - indataRange[0] ) / ( indataRange[1] - indataRange[0])
    data = data * (outdataRange[1] - outdataRange[0] ) + outdataRange[0]
    return data

def Image2array(o_img,img_x,img_y):
    imgArray = np.asarray(Image.open(o_img).convert('L').resize((img_x,img_y))).astype(np.float32)
    return imgArray
  
    
