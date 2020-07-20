from IPython.core.debugger import Tracer; keyboard = Tracer()
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from sklearn.decomposition import PCA

def plotPCA(img_comps, log_names, scat_shapes, mytitle, savefigName=None, scat_size=50):
    # asssuming img_comps is a list of ndarray (T, dim)
    
    # prepare all dataset of img_comps
    N = 0
    for img_comp in img_comps:
        N += img_comp.shape[0]
    
    traindata = np.zeros((N, img_comps[0].shape[1]))

    target = 0
    for img_comp in img_comps:
        traindata[target:target+img_comp.shape[0],:] = img_comp
        target += img_comp.shape[0]

    pca = PCA(n_components=2)
    pca.fit(traindata)

    colors = cm.jet(np.arange(0, len(img_comps))/float(len(img_comps)))

    for s, img_comp in enumerate(img_comps):
        img_comp_proj = pca.transform(img_comp)
        plt.scatter(img_comp_proj[:,0], img_comp_proj[:,1], marker=scat_shapes[s], s=scat_size, c=colors[s], label=log_names[s])

    contribution_values = pca.explained_variance_ratio_

    cv0 = '%2.2f' % contribution_values[0]
    cv1 = '%2.2f' % contribution_values[1]

    plt.xlabel('PCA 0 : '+cv0)
    plt.ylabel('PCA 1 : '+cv1)

    plt.title(mytitle)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)
    plt.grid()
    plt.subplots_adjust(right=0.7)

    if savefigName is not None:
        plt.savefig(savefigName)
    else:
        plt.show()
