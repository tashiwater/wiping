import numpy as np
from IPython.core.debugger import Tracer; keyboard = Tracer()

def data_formatter(dataset, shuffle=False):
    # concatenates all ndarray in list dataset to one ndarray
    # asssuming element of dataset is (n*dim) ndarray.
    

    # allocate memory
    r_mat = np.zeros((dataset.shape[0], dataset.shape[1]*dataset.shape[2])).astype(np.float32)
    # store the data
    ind=0
    for data in dataset:
        #print r_mat.shape
        #print dataset.shape[0],dataset.shape[1]*dataset.shape[2]
        r_mat[ind:ind+ind,:] = np.copy(data.reshape(dataset.shape[1]*dataset.shape[2]))
        ind += 1
        
    # shuffle order of the data
    if shuffle:
        s_ind = np.arange(dataset.shape[0])
        np.random.shuffle(s_ind)
        r_mat = r_mat[s_ind,:]

    return r_mat

    
    
