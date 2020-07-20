from IPython.core.debugger import Tracer; keyboard = Tracer()
import numpy as np
import matplotlib.pyplot as plt

def subplotImages(showFunc, images, titles, row, col, title, figsize=(8,6), TITLE_ONLY_TOP=False):
    """
    tiling images with given imshow function
    """
    ratio = max(figsize) / float(max(row,col))
    plt.figure(figsize=(col*ratio, row*ratio))
    
    count = 1
    for i in range(row):
        for j in range(col):
            if not count > len(images):
                plt.subplot(row,col,count)
                showFunc(images[count-1])
                plt.xticks(())
                plt.yticks(())
                if count <= col:
                    plt.title(titles[count-1])
                else:
                    if not TITLE_ONLY_TOP:
                        plt.title(titles[count])
                count += 1

    plt.suptitle(title, fontsize=14)
    plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9, wspace=0.1, hspace=0.1)

