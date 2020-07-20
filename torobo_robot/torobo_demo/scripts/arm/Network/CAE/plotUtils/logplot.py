import numpy as np
import matplotlib.pyplot as plt
from parse import search

def logplot(logFilename, myTitle='Training history', saveFigname=None):
    # parse log file and plot

    f = open(logFilename)
    lines = f.readlines()
    f.close()


    train_epoch = []
    train_loss = []
    test_epoch = []
    test_loss = []

    for line in lines:
        loss = search('loss_train = {}\n', line)
        if loss:
            epoch = search('[{:d}]', line)[0]
            train_epoch.append(epoch)
            train_loss.append(float(loss[0]))
        else:
            loss = search('loss_test = {}\n', line)
            if loss:
                epoch = search('[{:d}]', line)[0]
                test_epoch.append(epoch)
                test_loss.append(float(loss[0]))
            else:
                print '[WARNING] invalid format', line,
            
    train_epoch = np.array(train_epoch)
    train_loss = np.array(train_loss)

    test_epoch = np.array(test_epoch)
    test_loss = np.array(test_loss)

    plt.plot(train_epoch, train_loss/200, 'b-', label='Train loss')
    if test_epoch.shape[0]:
        plt.plot(test_epoch, test_loss, 'rx-', label='Test loss')
    plt.grid()
    plt.xlabel('epoch')
    plt.ylabel('Mean Sequare Error')
    plt.legend() 
    plt.title(myTitle)

    if saveFigname is not None:
        plt.savefig(saveFigname)
    else:
        plt.show()

#logplot("/home/assimilation/workspace_kim/Chainer/MTRNN/Result/2017_10_11_tanh_39_80_5_no_init_cs/rnntrain.log","result","Noinit.png")
