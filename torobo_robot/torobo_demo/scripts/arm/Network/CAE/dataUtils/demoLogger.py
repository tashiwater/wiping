import numpy as np
import time

class demoLogger:
    def __init__(self):
        self.pen_x = []
        self.pen_y = []
        self.img = []
        self.img_comp = []
        self.ang = []
        self.u_io = []
        self.u_cf = []
        self.u_cs = []
        self.io = []
        self.cf = []
        self.cs = []
        
    def start(self):
        self.start_time = time.time()
        self.time = [self.start_time]

    def logging(self, pen_x, pen_y, img, img_comp, ang, u_io, u_cf, u_cs, io, cf, cs):
        self.pen_x.append(pen_x)
        self.pen_y.append(pen_y)
        self.img.append(img)
        self.img_comp.append(img_comp)
        self.ang.append(ang)
        self.u_io.append(u_io)
        self.u_cf.append(u_cf)
        self.u_cs.append(u_cs)
        self.io.append(io)
        self.cf.append(cf)
        self.cs.append(cs)
        self.time.append(time.time()-self.start_time)
        
    def save(self, savename):
        np.savez(savename, pen_x = self.pen_x, \
                 pen_y = self.pen_y, \
                 img = self.img, \
                 img_comp = self.img_comp, \
                 ang = self.ang, \
                 u_io = self.u_io, \
                 u_cf = self.u_cf, \
                 u_cs = self.u_cs, \
                 io = self.io, \
                 cf = self.cf, \
                 cs = self.cs, \
                 time = self.time)
