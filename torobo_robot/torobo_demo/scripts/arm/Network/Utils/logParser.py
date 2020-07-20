from IPython.core.debugger import Tracer; keyboard = Tracer()
import numpy as np
import matplotlib.pyplot as plt
from parse import search
from datetime import datetime

def logParse(logFilename):

    f = open(logFilename)
    lines = f.readlines()
    f.close()

    train_epoch = []
    train_loss = []
    test_epoch = []
    test_loss = []
    calcTime_test = []
    calcTime_train = []

    start_time = -1

    for line in lines:
        
        strTime = search('{} - [', line)[0]
        
        timestamp = datetime.strptime(strTime, '%Y-%m-%d %H:%M:%S,%f')
        
        if start_time == -1:
            start_time = timestamp
            timeDiff = 0.0
        else:
            timeDiff = (timestamp - start_time).total_seconds()
        loss = search('loss_train = {}\n', line)
        if loss:
            epoch = search('[{:d}]', line)[0]
            train_epoch.append(epoch)
            train_loss.append(float(loss[0]))
            calcTime_train.append(timeDiff)
        else:
            loss = search('loss_test = {}\n', line)
            if loss:
                epoch = search('[{:d}]', line)[0]
                test_epoch.append(epoch)
                test_loss.append(float(loss[0]))
                calcTime_test.append(timeDiff)
            else:
                print '[WARNING] invalid format', line,

    calcTime_train = np.array(calcTime_train)
    calcTime_test = np.array(calcTime_test)

    train_epoch = np.array(train_epoch)
    train_loss = np.array(train_loss)

    test_epoch = np.array(test_epoch)
    test_loss = np.array(test_loss)

    return calcTime_train, calcTime_test, train_epoch, train_loss, test_epoch, test_loss
