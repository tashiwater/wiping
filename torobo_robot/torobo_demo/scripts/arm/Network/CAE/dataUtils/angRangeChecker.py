import numpy as np

class angRangeChecker:
    def __init__(self, angDim=5):
        self.angDim = angDim
        self.angRange = np.zeros((angDim, 2))
        self.angRange[:,0] = 999999 # initial min
        self.angRange[:,1] = -999999 # initial max

    def check(self, angles):
        for i in range(self.angDim):
            if angles[:,i].min() < self.angRange[i,0]:
                self.angRange[i,0] = angles[:,i].min()
            if angles[:,i].max() > self.angRange[i,1]:
                self.angRange[i,1] = angles[:,i].max()
