import chainer
from chainer import Variable, cuda
import chainer.functions as F
import chainer.links as L
import numpy as np

from IPython.core.debugger import Tracer; keyboard = Tracer()

class InitialState(chainer.Link):
	def __init__(self, n_size):
		super(InitialState, self).__init__(
			state = (1, n_size),
		)
		self.state.data = np.zeros((1, n_size)).astype(np.float32)
	def __call__(self):
		return self.state

class MTRNN(chainer.Chain):
	"""
	an implementation of multiple timescale recurrent neural networks
	"""
	def __init__(self, dims, tau, num_seq, seed=1):
		super(MTRNN,self).__init__()
		np.random.seed(seed)

		self.num_seq = num_seq
		self.dims = dims
		self.tau = tau

		# register parameters
		self.add_link('io2io', L.Linear(self.dims[0], self.dims[0]))
		self.add_link('io2cf', L.Linear(self.dims[0], self.dims[1]))

		self.add_link('cf2io', L.Linear(self.dims[1], self.dims[0]))
		self.add_link('cf2cf', L.Linear(self.dims[1], self.dims[1]))
		self.add_link('cf2cs', L.Linear(self.dims[1], self.dims[2]))

		self.add_link('cs2cf', L.Linear(self.dims[2], self.dims[1]))
		self.add_link('cs2cs', L.Linear(self.dims[2], self.dims[2]))

		# initialize initial values of slow context layer
			
		for s in range(num_seq):
			self.add_link('init_cs'+str(s), InitialState(self.dims[2])) #cs
		
	def forward(self, indata, init_cs, input_param=1.0, gen=False):
		if gen:

			indata = indata.astype(np.float32)
			init_cs = Variable(init_cs.astype(np.float32))


			io_buf = np.zeros(indata.shape)
			cf_buf = np.zeros((indata.shape[0], self.dims[1]))
			cs_buf = np.zeros((indata.shape[0], self.dims[2]))


		loss = 0.0
		for t in range(indata.shape[0]-1):
			if t == 0:
				u_io = Variable(np.zeros((1, self.dims[0])).astype(np.float32))
				u_cf = Variable(np.zeros((1, self.dims[1])).astype(np.float32))
				u_cs = init_cs
				#u_cs = Variable(np.zeros((1, self.dims[2])).astype(np.float32))
				cf = F.tanh(u_cf)
				cs = F.tanh(u_cs)

			if t == 0:
				x = Variable(indata[t,:].reshape(1, self.dims[0]))
			else:
				if not gen:
				#Closed loop
					mot= input_param * indata[t,0:6].reshape(1, 6) + (1.0-input_param) * io.data[0,0:6]
					gripper=(input_param) * indata[t,6:7].reshape(1, 1) + (1.0-input_param) * io.data[0,6:7]
					#img= (input_param) * indata[t,7:37].reshape(1, 30) + (1.0-input_param) * io.data[0,7:37]
					#label= (input_param) * indata[t,37:].reshape(1, 1) + (1.0-input_param) * io.data[0,37:]

				#Open loop
					#mot= (1.0-input_param) * indata[t,0:6].reshape(1, 6) + (input_param) * io.data[0,0:6]
					#gripper=(1.0-input_param) * indata[t,6:7].reshape(1, 1) + (input_param) * io.data[0,6:7]
					img= (1.0-input_param) * indata[t,7:37].reshape(1, 30) + input_param * io.data[0,7:37]
					label= (1.0-input_param) * indata[t,37:].reshape(1, 1) + input_param * io.data[0,37:]
				

					x=np.append(mot,gripper,axis=1)
					x=np.append(x,img,axis=1)
					x=np.append(x,label,axis=1)
					x= Variable(x)
				elif gen:
				#Closed loop
					mot= input_param * indata[t,0:6].reshape(1, 6) + (1.0-input_param) * io.data[0,0:6]
					gripper=(input_param) * indata[t,6:7].reshape(1, 1) + (1.0-input_param) * io.data[0,6:7]
					#img= (input_param) * indata[t,7:37].reshape(1, 30) + (1.0-input_param) * io.data[0,7:37]
					#label= (input_param) * indata[t,37:].reshape(1, 1) + (1.0-input_param) * io.data[0,37:]

				#Open loop
					#mot= (1.0-input_param) * indata[t,0:6].reshape(1, 6) + (input_param) * io.data[0,0:6]
					#gripper=(1.0-input_param) * indata[t,6:7].reshape(1, 1) + (input_param) * io.data[0,6:7]
					img= (1.0-input_param) * indata[t,7:37].reshape(1, 30) + input_param * io.data[0,7:37]
					label= (1.0-input_param) * indata[t,37:].reshape(1, 1) + input_param * io.data[0,37:]
				

					x=np.append(mot,gripper,axis=1)
					x=np.append(x,img,axis=1)
					x=np.append(x,label,axis=1)
					x= Variable(x)

			y = Variable(indata[t+1,:].reshape(1, self.dims[0]))
	
			u_io, u_cf, u_cs,  io, cf, cs = self.forward_partial(u_io, u_cf, u_cs, x, cf, cs)

			if gen:
				if t==0:
					io_buf[t,:]=indata[t,:]
					cf_buf[t,:]=np.zeros(self.dims[1])
					cs_buf[t,:]=init_cs.data
				io_buf[t+1,:] = io.data
				cf_buf[t+1,:] = cf.data
				cs_buf[t+1,:] = cs.data
			loss += F.mean_squared_error(io, y)
		if gen:
			return [io_buf, cf_buf, cs_buf], loss
		else:
			return loss

	def forward_partial(self, u_io, u_cf, u_cs, x, cf, cs):
		u_io = (1.0 - 1.0/self.tau[0]) * u_io + 1.0 / self.tau[0] * (self.__getitem__('io2io')(x) + self.__getitem__('cf2io')(cf))
		u_cf = (1.0 - 1.0/self.tau[1]) * u_cf + 1.0 / self.tau[1] * (self.__getitem__('cf2cf')(cf) + self.__getitem__('io2cf')(x)  + self.__getitem__('cs2cf')(cs))
		u_cs = (1.0 - 1.0/self.tau[2]) * u_cs + 1.0 / self.tau[2] * (self.__getitem__('cs2cs')(cs) + self.__getitem__('cf2cs')(cf)) 
		io = F.tanh(u_io)
		cf = F.tanh(u_cf)
		cs = F.tanh(u_cs)
		return u_io, u_cf, u_cs, io, cf, cs

	def get_InitialState(self, num):
		# get initial state of num's sequence
		return self.__getitem__('init_cs'+str(num)).state

