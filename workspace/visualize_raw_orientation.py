import numpy as np
import matplotlib.pyplot as plt
import time

fig = plt.figure()
plt.ion()
#plt.legend(loc='upper left')
plt.show()

poses = np.array([])


while True:
    # with open('src/isam2/data/estimate2.txt') as f:
    with open("chipmunk.txt") as f:
        lines = f.readlines() # list containing lines of file

    # Don't do anything if the file is empty
    if len(lines) == 0:
        continue

    for line in lines:
        line =  line[1:len(line)-2] #  line.strip('()') # remove leading/trailing white spaces
        if line:
            line = np.array(line.split(','))
            line = line.astype(float)
            if(poses.size == 0):
                poses = line
            else:
                poses = np.vstack((poses,line))
    

    if ((poses.ndim==2) and (poses.shape[0]) != 0) and ((poses.shape[0]) != 0):
        r = .2
        changesx = r*np.cos(poses[:,2]) 
        changesy = r*np.sin(poses[:,2]) 
        # # function to add arrow on a graph
        for i in range(len(changesx)):
            scatter = plt.arrow(poses[:,0][i],poses[:,1][i],changesx[i],changesy[i],width=0.05)
        plt.pause(.1)
