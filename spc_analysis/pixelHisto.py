#script to make a histogram of pixel values from a 2D binary array of 16-bit 
#ints. 

import matplotlib.pyplot as plt
import numpy as np
import argparse

if __name__=='__main__': 
    parser = argparse.ArgumentParser()
    parser.add_argument('inName', help='binary file containing array of 16-bit ints to histogram')
    parser.add_argument('--outName', '-o', help='binary file containing array of 16-bit ints to histogram')

    args = parser.parse_args()
    if args.outName: 
        outname = args.outName
    else:
        outName = inName + '_plain_histogram.hist'
    
    np.savetxt(outname, plainHisto('inName'))
    
def plainHisto(fname): 
    rdat = np.fromfile(fname, dtype='int16')
    datMax, datMin = np.max(rdat), np.min(rdat)
    nbins = datMax - datMin
    x = np.linspace(datMin, datMax - 1, datMax - datMin)
    hist = np.histogram(rdat, bins=nbins)[0]
    return np.array([x, hist])


