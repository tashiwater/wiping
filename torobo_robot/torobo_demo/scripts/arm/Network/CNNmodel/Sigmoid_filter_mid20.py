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
			conv1 = L.Convolution2D(inchannel, 32, 32, stride=1, pad=2, wscale=0.02*math.sqrt(32*32)),
			bn1 = L.BatchNormalization(32),
			conv2 = L.Convolution2D(32, 64, 9, stride=2, pad=1, wscale=0.02*math.sqrt(9*9*32)),
			bn2 = L.BatchNormalization(64),
			conv3 = L.Convolution2D(64, 128, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*64)),
			bn3 = L.BatchNormalization(128),
			conv4 = L.Convolution2D(128, 256, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*128)),
			bn4 = L.BatchNormalization(256),
			conv5 = L.Convolution2D(256, 512, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*256)),
			bn5 = L.BatchNormalization(512),
			l6 = L.Linear(512*6*4, 254),
			bn6 = L.BatchNormalization(254),
			l7 = L.Linear(254,20),
			bn7 = L.BatchNormalization(20),
			l8 = L.Linear(20,254),
			bn8 = L.BatchNormalization(254),
			l9 = L.Linear(254, 512*6*4),
			bn9 = L.BatchNormalization(512*6*4),
			dconv10 = L.Deconvolution2D(512, 256, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*256)),
			bn10 = L.BatchNormalization(256),
			dconv11 = L.Deconvolution2D(256, 128, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*128)),
			bn11 = L.BatchNormalization(128),
			dconv12 = L.Deconvolution2D(128, 64, 4, stride=2, pad=1, wscale=0.02*math.sqrt(4*4*64)),
			bn12 = L.BatchNormalization(64),
			dconv13 = L.Deconvolution2D(64, 32, 9, stride=2, pad=1, wscale=0.02*math.sqrt(9*9*32)),
			bn13 = L.BatchNormalization(32),
			dconv14 = L.Deconvolution2D(32, inchannel, 32, stride=1, pad=2, wscale=0.02*math.sqrt(32*32)),
		)


	def encode(self, x, test):
		#noise_factor=0.2
		#x=x+noise_factor*np.random.normal(0.,1.,)
		#print x[0][0].data
		hid = F.relu(self.bn1(self.conv1(x), test=test))
		hid = F.relu(self.bn2(self.conv2(hid), test=test))
		hid = F.relu(self.bn3(self.conv3(hid), test=test))
		hid = F.relu(self.bn4(self.conv4(hid), test=test))
		hid = F.relu(self.bn5(self.conv5(hid), test=test))
		hid = F.relu(self.bn6(self.l6(hid), test=test))
		hid = F.sigmoid(self.bn7(self.l7(hid), test=test)) 

		return hid
	

	def decode(self, x, test):
		hid = F.relu(self.bn8(self.l8(x), test=test))
		I9=F.relu(self.bn9(self.l9(hid), test=test))
		hid = F.reshape(I9, (x.data.shape[0], 512, 4, 6))
		hid = F.relu(self.bn10(self.dconv10(hid), test=test))
		hid = F.relu(self.bn11(self.dconv11(hid), test=test))
		hid = F.relu(self.bn12(self.dconv12(hid), test=test))
		hid = F.relu(self.bn13(self.dconv13(hid), test=test))
		hid = F.relu(self.dconv14(hid))
		
		return hid
		
	def __call__(self, x, train=True):
		test = not train
		if train==2:
			test=True
			return self.decode(self.encode(x,test),test)
		elif train==3:
			test=True
			return self.decode(x,test)
		elif test:
			return self.encode(x,test)
		elif train:
			return self.decode(self.encode(x,test),test)
