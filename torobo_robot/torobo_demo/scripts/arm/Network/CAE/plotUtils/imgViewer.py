from IPython.core.debugger import Tracer; keyboard = Tracer()
import matplotlib.pyplot as plt
import numpy as np

"""
imshow image data
"""
def imgViewer(ax, img, valueRange):
    # valueRange -> [0.0, 1.0]
    ax.imshow(img, cmap='gray',vmin=valueRange[0],vmax=valueRange[1],interpolation='none')
