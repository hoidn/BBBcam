import argparse
import os

"""Perform cluster analysis and transfer files to remote host"""

def main(dimx, dimy, thresh, host, path, expfname, darkfname = None, \
        transfer = False, analysis = True):
    if analysis == False: 
        assert transfer == True, "Must transfer .bin file(s) to remote host or\
 perform cluster analysis locally"
    filesStr = '' #string with whitespace-delimited names of files to transfer
    if analysis:
        clusterExec = './clusters ' + dimx + ' ' + dimy + ' ' + thresh + ' ' + \
            expfname
        if darkfname != None: 
            clusterExec += ' ' + darkfname
        os.system(clusterExec)
        filesStr += expfname + '_clusters.hist ' + expfname + '_pixels.hist'
    #TODO: convert the .hist files to a text format
    if transfer:
        filesStr += ' ' + expname
        if darkfname != None: 
            filesStr += ' ' + darkfname
    #transfer the files
    os.system('scp ' + filesStr + ' ' + host + ':' + path)
    if not analysis:
        #NOTE: clusters needs to be on the path on the remote machine
        os.system('ssh ' + host + " cd " + path + "; clusters")
        
        

if __name__ == '__main__': 
    a_parser = argparse.ArgumentParser()
    a_parser.add_argument("dimx", help = "Vertical dimension of the image")
    a_parser.add_argument("dimy", help = "Horizontal dimension of the image")
    a_parser.add_argument("threshold", help = "Noise threshold for the cluster analysis routine")
    a_parser.add_argument("hostPath", help = "user@remoteHost:path")
    a_parser.add_argument("exposure", help = "Name of the x-ray exposure .bin file")
    a_parser.add_argument("--dark", help = "Name of the dark frame .bin file")
    a_parser.add_argument("-t", "--transfer",  help = "Transfer image data .bin files in addition to the .hist files resulting from cluster analysis", action = 'store_true')
    a_parser.add_argument("-n", "--noanalysis",  help = "Perform cluster analysis on the remote machine instead of the Raspberry Pi. Requires --transfer", action = 'store_true')
    args = a_parser.parse_args()
    [host, path] = args.hostPath.split(":")
    main(args.dimx, args.dimy, args.threshold, host, path, args.exposure, args.dark, args.transfer, not args.noanalysis)
