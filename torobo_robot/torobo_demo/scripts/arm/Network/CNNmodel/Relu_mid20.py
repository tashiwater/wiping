import math
import numpy as np
import chainer
import chainer.functions as F
import chainer.links as L
from IPython.core.debugger import Tracer; keyboard = Tracer()

class DCNNAE(chainer.Chain):
	"""
	An implementation of deep convolutional neural network autoencoders
	"""
	def __init__(self, inchannel=3, seed = 1):
		
		np.random.seed(seed)

		super(DCNNAE, self).__init__(
			conv1 = L.Convolution2D(inchannel, 32, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4)),
			bn1 = L.BatchNormalization(32),
			conv2 = L.Convolution2D(32, 64, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*32)),
			bn2 = L.BatchNormalization(64),
			conv3 = L.Convolution2D(64, 128, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*64)),
			bn3 = L.BatchNormalization(128),
			conv4 = L.Convolution2D(128, 256, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*128)),
			bn4 = L.BatchNormalization(256),
			l5 = L.Linear(256*4*3, 254),
			bn5 = L.BatchNormalization(254),
			l6 = L.Linear(254,20),
			bn6 = L.BatchNormalization(20),
			l7 = L.Linear(20,254),
			bn7 = L.BatchNormalization(254),
			l8 = L.Linear(254, 256*4*3),
			bn8 = L.BatchNormalization(256*4*3),
			dconv9 = L.Deconvolution2D(256, 128, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*128)),
			bn9 = L.BatchNormalization(128),
			dconv10 = L.Deconvolution2D(128, 64, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*64)),
			bn10 = L.BatchNormalization(64),
			dconv11 = L.Deconvolution2D(64, 32, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*32)),
			bn11 = L.BatchNormalization(32),
			dconv12 = L.Deconvolution2D(32, inchannel, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4)),
		)


	def encode(self, x, test):
		hid = F.relu(self.bn1(self.conv1(x), test=test))
		hid = F.relu(self.bn2(self.conv2(hid), test=test))
		hid = F.relu(self.bn3(self.conv3(hid), test=test))
		hid = F.relu(self.bn4(self.conv4(hid), test=test))
		hid = F.relu(self.bn5(self.l5(hid), test=test))
		hid = F.relu(self.bn6(self.l6(hid), test=test)) # 30dim

		return hid


	def decode(self, x, test):
		hid = F.relu(self.bn7(self.l7(x), test=test))
		hid = F.reshape(F.relu(self.bn8(self.l8(hid), test=test)), (x.data.shape[0], 256, 3, 4))
		hid = F.relu(self.bn9(self.dconv9(hid), test=test))
		hid = F.relu(self.bn10(self.dconv10(hid), test=test))
		hid = F.relu(self.bn11(self.dconv11(hid), test=test))
		hid = F.relu(self.dconv12(hid))
		
		return hid
		
	def __call__(self, x, train=True):
		test = not train
		if train==2:
			test=True
			return self.decode(self.encode(x,test),test)
		elif test:
			return self.encode(x,test)
		elif train:
			return self.decode(self.encode(x,test),test)
