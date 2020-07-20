import numpy as np
import os
import chainer
import chainer.functions as F
import chainer.links as L
from chainer import optimizers
from chainer import serializers

import logging
from logging import FileHandler

from MTRNN import MTRNN

from IPython.core.debugger import Tracer; keyboard = Tracer()

def RNNtrain(model, optimizer, dataset, train_params, pool=None):
	# train_param['input_param'] : input parameter in the io neurons
	INPUT_PARAM = train_params['input_param']
	DECAY = train_params['decay']
	EPOCH = train_params['epoch']
	PRINT_ITER = train_params['print_iter']
	SNAP_ITER = train_params['snap_iter']
	SNAP_DIR = train_params['snap_dir']
	LOG_NAME = train_params['log_name']

	if not os.path.exists(SNAP_DIR):
		print '[ERROR]', SNAP_DIR, ' does not exist.'
		return None # exit this function

	num_seq = len(dataset)
	if SNAP_DIR[-1] != '/':
		SNAP_DIR += '/'

	# init logger
	stream_logger = logging.StreamHandler()
	stream_logger.setLevel(logging.INFO)
	stream_logger.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))

	file_logger = logging.FileHandler(filename=LOG_NAME, mode='w')
	file_logger.setLevel(logging.INFO)
	file_logger.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
	
	logging.getLogger().addHandler(stream_logger)
	logging.getLogger().addHandler(file_logger)
	logging.getLogger().setLevel(logging.INFO)

	# convert to float32
	"""
	dataset_ = []
	for data in dataset:
		dataset_.append(pre_process(data))
	"""
	
	optimizer.setup(model)
	for epoch in range(1, EPOCH+1):
		dataset2=pre_process(dataset)
		if not pool is None:
			loss_val = epoch_multiProcess(pool, model, optimizer, num_seq, dataset_, INPUT_PARAM, DECAY)
		else:
			loss_val = epoch_singleProcess(model, optimizer, num_seq, dataset2, INPUT_PARAM, DECAY)
		if epoch % PRINT_ITER == 0:
			logging.info('['+str(epoch)+'] loss_train = '+str(loss_val))
		if epoch % SNAP_ITER == 0:
			#model._children=children_name[0]
			serializers.save_npz(SNAP_DIR+'iter_'+str(epoch)+'.rnnmodel', model)
			#model._children=children_name[1]

def epoch_multiProcess(pool, model, optimizer, num_seq, dataset, input_param, decay):
	
	model.zerograds()
			
	args = []
	for s in range(num_seq):
		args.append((s, model, dataset[s], input_param))

	grad_loss = pool.map(forward_backward_multiProcess, args)
		
	loss_val = 0.0
	for s in range(num_seq):
		unpack_grads(model, grad_loss[s][0])
		loss_val += grad_loss[s][1]

	optimizer.update()
	#keyboard()
	return loss_val

def forward_backward_multiProcess(arg):
	
	s = arg[0]
	model = arg[1]
	data = arg[2]
	input_param = arg[3]
	model.zerograds()
	loss = model.forward(data, model.get_InitialState(s), input_param)
	loss.backward()
	
	# pack all gradients of model because model is not picklable object 
	grads = pack_grads(model)
	return [grads, loss.data]

def pack_grads(model):
	# store all gradients of weight
	grads = []
	for l in model.links():
		for p in l.params():
			grads.append(p.grad)
	return grads

def unpack_grads(model, grads):
	# unpack grads and accumulate to model
	g_idx = 0
	for l in model.links():
		for p in l.params():
			p.grad += grads[g_idx]
			g_idx += 1

def epoch_singleProcess(model, optimizer, num_seq, dataset, input_param, decay):
	model.zerograds()
	loss = 0.0
	
	loss = model.forward(dataset, model.get_InitialState(), input_param)
	loss.backward()
	optimizer.weight_decay(decay)
	optimizer.update()

	#keyboard()
	return loss.data

def GN(data, var=0.01):
	noise = np.random.normal(scale=var, size=data.shape)
	noise = noise.astype(np.float32)
	return data+noise

def pre_process(data):
	data=GN(data)
	data=data.astype(np.float32)
	return data
