from IPython.core.debugger import Tracer; keyboard = Tracer()
import argparse
import os,glob

def printArgs(parser):
	print '#---------- Arguments ----------#'
	for x in vars(parser):
		print x, ' = ', getattr(parser, x)
	print '#--------------------------------# \n'

def checkPathOrDie(path):
	path_checked=glob.glob(path)
	for file in path_checked:
		if not os.path.exists(file):
			print file, ' does not exists!'
			raise ValueError
		if file[-1] is '/':
			file = file[:-1] # cut "/"
	return path

def saveArgLogs(parser, filename):
	with open(filename, 'w') as f:
		for x in vars(parser):
			strline =  str(x) + ' = ' + str(getattr(parser, x)) + '\n'
			f.write(strline)
	pass

#checking the loaded files that are raw image or zipped array file
def checkFileType(path):
	
	for check in path:
		if check.split(".")[-1]=="npz":
			raw_image=False
			break
		else:
			raw_image=True
			break
	print path
	return raw_image
