import numpy as np
import sys
from matplotlib import cm
import matplotlib.pyplot as plt

def main(prefix):
    name= prefix  + 'sum.dat'

    arr =np.reshape(np.fromfile(name, dtype = 'uint32'), (1024, 1280))
    plt.imshow(arr, interpolation='none', cmap = cm.Greys_r)
    plt.savefig(name[:-3] + 'png')

prefix = sys.argv[1]
main(prefix)
