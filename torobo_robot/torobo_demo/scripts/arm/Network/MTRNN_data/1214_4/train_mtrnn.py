from IPython.core.debugger import Tracer; keyboard = Tracer()

# add path of the root directory
import sys,os,re,shutil
sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/..')

from MTRNN.MTRNN import MTRNN
from MTRNN.MTRNN import InitialState
from MTRNN.RNNtrainer import RNNtrain

import numpy as np
from chainer import optimizers, serializers
#from multiprocessing import Pool

Output_dir = 'Result/result1216/'

if not os.path.exists(Output_dir):
	os.makedirs(Output_dir)

shutil.copy("./MTRNN/MTRNN.py",Output_dir)
shutil.copy("./MTRNN/RNNtrainer.py",Output_dir)
shutil.copy(__file__,Output_dir)


dataset=np.load("/home/assimilation/Desktop/DCNNAE/Result/2019_12_16_Sigmoid_20/Image_extract_1000/combine.npy")

Io=46
Cf=100
Cs=10

model = MTRNN([Io,Cf,Cs], [2,5,50], dataset.shape[0], seed=1)

model.add_link('init_cs', InitialState(dataset.shape[0], Cs)) #cs
optimizer = optimizers.Adam()

#input param ==> 0.0 = closed loop 1.0 = open loop
train_params = {'input_param': 0.1, \
                'decay': 0.00001, \
                'epoch': 200000, \
                'print_iter': 10, \
                'snap_iter': 50, \
                'snap_dir': Output_dir, \
                'log_name': Output_dir+"rnntrain.log"}

RNNtrain(model, optimizer, dataset, train_params, pool=None)
