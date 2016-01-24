import sys
import subprocess
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('threshold', nargs = 1, help = 'threshold value for cluster analysis')
parser.add_argument('--o', '-o', nargs = 1,  help = "name of output file")
parser.add_argument('--d', '-d',  help = "name of dark frame subtraction file")
parser.add_argument('--g', '-g',  help = "gain")
parser.add_argument('--n', '-n',  help = "number of exposures")
parser.add_argument('--r', '-r', nargs = 2,  help = "range of values considered to construct sum frame and 2d istogram")

args = parser.parse_args()

lock_file = '/var/lock/imx291_cam'

def check_sensor_running():
    return os.path.isfile(lock_file)

def targetGain():
    targetGain = args.g
    if not targetGain: # is None
        targetGain = '0x7f' # default gain
    return targetGain

def run_cam(i2c_config = True, cam_config = False):
    argStr = scan_out_gain()
    if i2c_config:  
        argStr += ' ' + targetGain()
    if cam_config:
        argStr += ' -c'
    return argStr

# return console input parameters without the -g option, if it was provided
def scan_out_gain():
    newList = []
    args = sys.argv[2:]
    i = 0
    while i < len(args):
        if args[i] == '-g':
            i += 1
        else:
            newList += [args[i]]
        i += 1

    print ' '.join(newList)
    return ' '.join(newList)

    
if check_sensor_running():
    # check gain value
    p = subprocess.Popen("sudo i2cget -y 1 0x5d 0x35 w".split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    currentGain = out[:-1] # current sensor gain in hex string form
    if targetGain() == currentGain:
        run_cam(i2c_config = False, cam_config = False)
    else:
        run_cam(i2c_config = True, cam_config = False)

else:
    run_cam(i2c_config = True, cam_config = True)
    os.system("touch  " + lock_file)


