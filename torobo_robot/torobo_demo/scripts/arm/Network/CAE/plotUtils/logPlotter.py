from IPython.core.debugger import Tracer; keyboard = Tracer()
import numpy as np
import matplotlib.pyplot as plt

"""
plots logfile from DrawingLogger.
Assuming log file format: timestamp, angle(6), x,y,p
"""
def logPlotter(logfile, figtitle, savename):
    data = np.loadtxt(logfile)

    timestamp = data[:,0]
    angle = data[:,1:6]
    x = data[:,7]
    y = data[:,8] * -1.0

    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(15,6))
    
    
    ax1.plot(x,y)
    ax1.scatter(x[0], y[0], s=100, c='blue')
    ax1.set_title('X-Y of pen tablet')
    ax1.grid()
    
    ax2.plot(timestamp, angle)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Angle [rad]')
    ax2.set_title('Angle')
    ax2.grid()

    f.suptitle(figtitle, fontsize=16)
    plt.tight_layout()

    plt.savefig(savename)

    
    
    
