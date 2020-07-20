import multiprocessing
import os
import struct

class TabletReader(multiprocessing.Process):
    def __init__(self, eventFile):
        pen_state = multiprocessing.Array('i', [0,0,0])
        multiprocessing.Process.__init__(self, args=(pen_state,))
        self.pen_state = pen_state
        self.exit = multiprocessing.Event()
        
        # open device file
        self.eventsize = struct.calcsize('llHI')
        self.device_f = open(eventFile, 'rb')
        
        self.event = self.device_f.read(self.eventsize)
        print 'device file loaded.'

    def get(self):
        return self.pen_state[:]

    def __del__(self):
        # close device file
        self.device_f.close()
        
    def run(self):
        while not self.exit.is_set():
            (tv_sec, tv_usec, dtype, code, value) = struct.unpack('llHHI', self.event)
            if dtype != 0 or code != 0 or value != 0:
                if dtype == 3: #EV_ABS
                    if code == 0: #ABS_X
                        self.pen_state[0] = value
                    elif code == 1: #ABS_Y
                        self.pen_state[1] = value
                    elif code == 24: #ABS_PRESSURE
                        self.pen_state[2] = value
            # read device file
            self.event = self.device_f.read(self.eventsize)
        
    def shutdown(self):
        print 'shutdown TabletReader ... '
        self.exit.set()



