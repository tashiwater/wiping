from IPython.core.debugger import Tracer; keyboard = Tracer()
import glob,re
import numpy as np
from PIL import Image

def normalization(data, indataRange, outdataRange):
    data = ( data - indataRange[0] ) / ( indataRange[1] - indataRange[0] )
    data = data * ( outdataRange[1] - outdataRange[0] ) + outdataRange[0]
    return data

def loadGrayImage(filename, imsize_y, imsize_x,norm=[0,255]):
    """
    returns array(1, W, H)
    """
    img = np.asarray(Image.open(filename).convert('L').resize((imsize_x,imsize_y))).reshape(1,imsize_y,imsize_x).astype(np.float32)
    #img =Image.open(filename).convert('L').resize((imsize_x,imsize_y))
    #img.show()
    img = normalization(img, [0,255], norm)
    return img

def loadColorImage(filename, imsize_y, imsize_x, norm=[0,255]):
    """
    returns array(3, W, H)
    """
    img = np.asarray(Image.open(filename).convert('RGB').resize((imsize_x,imsize_y))).astype(np.float32)
    img = normalization(img, [0,255], norm)
    return img.transpose((2,0,1))

def get_batch(batch_size, loop, batch, data): 
	try:
		batch[0:batch_size,:,:,:] = data[loop:loop+batch_size,:,:,:]
	except:
		batch=data[loop:loop+batch_size,:,:,:]

def sort_nicely(list):
	convert = lambda text: int(text) if text.isdigit() else text 
	alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
	list.sort( key=alphanum_key ) 

def Data_transform(Data_path, Raw_image_check, imsize_y, imsize_x, inchannel, norm):
	
	Data=[]

	if Raw_image_check:
		if inchannel==1:
			for i in Data_path:
				Data.append(loadGrayImage(i, imsize_y, imsize_x,norm))
			return np.array(Data)

		elif inchannel==3:
			for i in Data_path:
				Data.append(loadColorImage(i, imsize_y, imsize_x,norm))
			return np.array(Data)
	
	else:
		for i in Data_path:
			Data=np.load(i)['imgs']
		return Data

