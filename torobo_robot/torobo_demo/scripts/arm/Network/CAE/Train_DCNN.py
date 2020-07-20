from IPython.core.debugger import Tracer; keyboard = Tracer()

# add path of the root directory
import sys,os
import argparse
import threading
from six.moves import queue
import numpy as np
import time
import logging
import datetime
import subprocess
import glob
import re

from Utils.dataUtils import *
from Utils.argUtils import *
from Utils.logUtils import *
from CNNmodel.DCNNAE import DCNNAE

import chainer
from chainer import Variable
from chainer import optimizers
from chainer import cuda
from chainer import functions
from chainer import serializers

# checking cuda environment
cuda.check_cuda_available()
from chainer.cuda import cupy as xp

def GN(data, var=1):
	###adding noise to input image	
	noise = np.random.normal(scale=var, size=data.shape)
	noise = noise.astype(np.float32)
	return data+noise

def manage():
	train_batch = np.ndarray((args.batchsize_train, args.inchannel, args.imsize_y, args.imsize_x),dtype=np.float32)
	test_batch = np.ndarray((args.batchsize_test, args.inchannel, args.imsize_y, args.imsize_x),dtype=np.float32)
	np.random.shuffle(train_f)
	for iteration in range(0, args.maxiter):
		# shuffle train data
		

		if iteration == 0 or iteration % args.testiter == 0:
			# shuffle test data
			np.random.shuffle(test_f)
			main_q.put(('test', iteration))
			
			# do test	
			for i in range(0, test_f.shape[0], args.batchsize_test):
				get_batch(args.batchsize_test, i, test_batch, test_f)
				main_q.put((test_batch, iteration))

			main_q.put(('train', iteration))

		#do train
		for i in range(0, train_f.shape[0], args.batchsize_train):
		
			#get_batch(args.batchsize_train, int(np.random.rand()*(train_f.shape[0]-args.batchsize_train)//1), train_batch, train_f)
			
			get_batch(args.batchsize_train, i , train_batch, train_f)		
		
			main_q.put((train_batch, iteration))
		
	main_q.put(('test', iteration))
	# at the end of training trainer does test once
	for i in range(0, test_f.shape[0], args.batchsize_test):
		get_batch(args.batchsize_test, i, test_batch, test_f)
		main_q.put((test_batch, iteration))

	main_q.put(('end', args.maxiter))

def mylogging():
	flag_log_train = True
	count=0
	sum_loss=0.0
	while True:
		result = log_q.get()
		if result[0] is 'end':
			logging.info('['+str(test_iter_begin)+'] loss_test = '+str(test_loss/float(test_iter_count)))
			break

		elif result[0] is 'train':
			if not flag_log_train:
				logging.info('['+str(test_iter_begin)+'] loss_test = '+str(test_loss/float(test_iter_count)))
				
			if result[2] % args.printiter == 0:
				#use when update parameter after training all data as batch				
				#"""				
				count+=1	
				sum_loss+=result[1]
				if count%(train_f.shape[0]/args.batchsize_train)==0:
					logging.info('['+str(result[2])+'] loss_train = '+str(sum_loss/(train_f.shape[0]/args.batchsize_train)))
					sum_loss=0.0
				#"""
				"""
				#use when update parameter after training each batch
				logging.info('['+str(result[2])+'] loss_train = '+str(result[1]))
				"""
			flag_log_train = True
			continue
		
		elif result[0] is 'test':
			if flag_log_train:
				# test starts
				test_iter_count = 1
				test_loss = result[1].copy()
				test_iter_begin = result[2]
			else:
				# while testing
				test_iter_count += 1
				test_loss += result[1].copy()

			flag_log_train = False
			continue

def train():
	while True:
		while main_q.empty():
			time.sleep(0.01)

		inp = main_q.get()
		if inp[0] is 'end':
			log_q.put(('end', inp[1]))
			break

		elif inp[0] is 'train':
			train_mode = True
			continue

		elif inp[0] is 'test':
			# snap model
			snapnamebase = args.snap + '/snap_iter_' + str(inp[1])
			serializers.save_npz(snapnamebase+'.cnnmodel', model)
			#serializers.save_npz(snapnamebase+'.optimizerstate', optimizer)

			train_mode = False
			continue

		if train_mode:
			# do train
			#x = chainer.Variable(xp.asarray(inp[0]))
			#x = chainer.Variable(cuda.to_gpu(inp[0]), args.gpuid)
			x = chainer.Variable(cuda.to_gpu(xp.clip(GN(inp[0]),0.,255.)), args.gpuid)
			model.zerograds()
			y = model(x)
			loss = functions.mean_squared_error(y,x)
			loss.backward()
			optimizer.update()
			log_q.put(('train', loss.data.get(), inp[1]))

		else:
			# do test
			x = chainer.Variable(cuda.to_gpu(inp[0]), args.gpuid)
			y = model(x, train=2)
			loss = functions.mean_squared_error(y,x)
			log_q.put(('test', loss.data.get(), inp[1]))
		
		del x

parser = argparse.ArgumentParser(description='Learning CNN Autoencoder with multiGPU.')
parser.add_argument('--traindata', help='Path to training dataset')
parser.add_argument('--testdata', help='Path to testing dataset')
parser.add_argument('--snap', help='Path to snap models')
parser.add_argument('--log', help='Path to logging file name')
parser.add_argument('--batchsize_train', type=int, default=32, \
					help='Learning minibatch size')
parser.add_argument('--batchsize_test', type=int, default=32, \
					help='Learning minibatch size')
parser.add_argument('--maxiter', type=int, default=100, \
					help='Number of batch-wise iteration')
parser.add_argument('--testiter', type=int, default=100, \
					help='Interval of testing')
parser.add_argument('--printiter', type=int, default=100, \
					help='Interval of printing')
parser.add_argument('--inchannel', type=int, default=3, \
					help='Number of channel in images')
parser.add_argument('--imsize_x', type=int, default=64, \
					help='Size of images')
parser.add_argument('--imsize_y', type=int, default=64, \
					help='Size of images')
parser.add_argument('--decay', type=float, default=0.000001, \
					help='Weight decay')
parser.add_argument('--alpha', type=float, default=0.0002, \
					help='alpha(learning rate) of Adam')
parser.add_argument('--beta1', type=float, default=0.5, \
					help='beta1 of Adam')
parser.add_argument('--seed', type=int, default=1234, \
					help='Random seed for initialization of weights')
parser.add_argument('--minval', type=float, default=-1, \
					help='Minimum vale of input data')
parser.add_argument('--maxval', type=float, default=1, \
					help='Maximum vale of input data')
parser.add_argument('--gpuid', type=int, default=0, \
					help='ID of the first gpu to use')
args = parser.parse_args()

# check arguments
args.traindata = checkPathOrDie(args.traindata)
args.testdata = checkPathOrDie(args.testdata)
args.snap = checkPathOrDie(args.snap)
args.log = checkPathOrDie(args.log)
NORM = [args.minval, args.maxval]

# show arguments
printArgs(args)

# initialize models
model = DCNNAE(inchannel=args.inchannel, seed=args.seed)
cuda.get_device(args.gpuid).use()
model.to_gpu(args.gpuid)
optimizer = optimizers.Adam(alpha=args.alpha, beta1=args.beta1)
optimizer.setup(model) 
optimizer.add_hook(chainer.optimizer.WeightDecay(args.decay))

#load image and transpose for making training form
trainfiles = glob.glob(args.traindata)
testfiles = glob.glob(args.testdata)
"""
N: The number of data
C: Channel
W: Width 
H: Height
Trans form data array(N, C, W, H)
"""
print "trainfiles",trainfiles
train_f=Data_transform(trainfiles, checkFileType(trainfiles), args.imsize_y, args.imsize_x,args.inchannel, NORM)
test_f=Data_transform(testfiles, checkFileType(testfiles), args.imsize_y, args.imsize_x,args.inchannel, NORM)
del trainfiles,testfiles

# queues to communicate 
main_q = queue.Queue(maxsize=1)
log_q = queue.Queue(maxsize=1)
data_q = queue.Queue(maxsize=1)

########## main ####################################################
logfilenamebase = os.path.abspath(args.log + '/' + datetime.datetime.today().strftime("%Y%m%d%H%M%S")) # + .log or .xxx
logfilename_args = logfilenamebase + '.paramlog'
saveArgLogs(args, logfilename_args) # snap argument logs
subprocess.check_call(['ln', '-s', '-f', logfilename_args, args.log+'/sgdTrain.paramlog'])

print ' log file of arguments : ', args.log+'/sgdTrain.paramlog'

logfilename_train = logfilenamebase + '.trainlog'
initLogger(logfilename_train)
subprocess.check_call(['ln', '-s', '-f', logfilename_train, args.log+'/sgdTrain.trainlog'])

print ' log file of training : ', args.log+'/sgdTrain.trainlog\n'

mylogger = threading.Thread(target=mylogging)
mylogger.daemon = True
mylogger.start()

trainer = threading.Thread(target=train)
trainer.daemon = True
trainer.start()

manage()

mylogger.join()
trainer.join()

