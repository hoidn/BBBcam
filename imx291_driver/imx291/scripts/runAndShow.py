import os
import numpy as np
import sys
from matplotlib import cm

prefix = sys.argv[1]
name= prefix  + 'sum.dat'

def main(prefix):
    arr =np.reshape(np.fromfile(name, dtype = 'uint32'), (1024, 1280))
    #plt.imshow(arr, interpolation='none')
    plt.imshow(arr, interpolation='none', cmap = cm.Greys_r)
    plt.savefig('images/' + name[:-3] + 'png')
    plt.show()

os.system('time sudo ./run_imx291 1 -o ' + prefix + ' -n 40 -r 0 255' + name + ' .')
main(prefix)
