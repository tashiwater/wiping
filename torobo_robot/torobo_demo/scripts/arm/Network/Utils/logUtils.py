import logging
from logging import FileHandler

def initLogger(logfilename):
    stream_logger = logging.StreamHandler()
    stream_logger.setLevel(logging.INFO)
    stream_logger.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
    file_logger = logging.FileHandler(filename=logfilename, mode='w')
    file_logger.setLevel(logging.INFO)
    file_logger.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))
    logging.getLogger().addHandler(stream_logger)
    logging.getLogger().addHandler(file_logger)
    logging.getLogger().setLevel(logging.INFO)
