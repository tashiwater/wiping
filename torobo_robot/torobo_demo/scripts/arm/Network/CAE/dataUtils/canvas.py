from IPython.core.debugger import Tracer; keyboard = Tracer()
from PIL import Image, ImageDraw
import numpy as np

"""
utility class of canvas for converting between x-y axes and image 
"""
class canvas:
    def __init__(self, imsize, linewidth, xRange, yRange, valueRange):
        self.imsize = imsize
        self.linewidth = linewidth
        self.xRange = xRange
        self.yRange = yRange
        self.valueRange = valueRange

        self.resetCanvas()

    def resetCanvas(self):
        self.canvas = Image.new('L', (self.imsize, self.imsize), 'white')
        self.drawer = ImageDraw.Draw(self.canvas)

    def startDraw(self, x, y):
        self.x_, self.y_ = self.normalizeAxes(x,y)

    def draw(self, x, y):
        x, y = self.normalizeAxes(x,y)
        self.drawer.line((self.x_, self.y_, x, y), fill='black', width=self.linewidth)
        self.x_ = x
        self.y_ = y

    def getImgArray(self):
        imgarray = np.array(self.canvas.getdata()).reshape(self.imsize, self.imsize)
        return self.normalizeImg(imgarray)

    def drawLineSeq(self, xSeq, ySeq):
        # buffer fot storing image sequence
        img = np.zeros((xSeq.shape[0], self.imsize, self.imsize))

        self.resetCanvas()        
        self.startDraw(xSeq[0], ySeq[0])
        for t in range(xSeq.shape[0]):
            self.draw(xSeq[t], ySeq[t])
            img[t,:,:] = self.getImgArray()
        return img

    def normalizeAxes(self, x, y):
        r_x = ( x - self.xRange[0] ) / float( self.xRange[1] - self.xRange[0] ) * self.imsize
        r_y = ( y - self.yRange[0] ) / float( self.yRange[1] - self.yRange[0] ) * self.imsize
        return r_x, r_y

    def normalizeImg(self, img):
        return ( img * -1.0 + img.max() ) / (img.max() - img.min()) * \
            (self.valueRange[1] - self.valueRange[0]) + self.valueRange[0]
        
