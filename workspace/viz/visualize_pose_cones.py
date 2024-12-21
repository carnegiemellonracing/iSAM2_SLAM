import numpy as np
import matplotlib.pyplot as plt
import time

fig = plt.figure()
plt.ion()
#plt.legend(loc='upper left')
plt.show()

poses = np.array([])
cones = np.array([])


while True:
    # with open('src/isam2/data/estimate2.txt') as f:
    with open("urmom.txt") as f:
        lines = f.readlines() # list containing lines of file

    # Don't do anything if the file is empty
    if len(lines) == 0:
        continue
    for line in lines:
        line =  line[1:len(line)-2] #  line.strip('()') # remove leading/trailing white spaces
        if line:
            line = np.array(line.split(','))
            line = line.astype(float)
            if(len(line) == 2):
                if(cones.size == 0):
                    cones = line
                else:
                    cones = np.vstack((cones,line))
            if(len(line) == 3):
                if(poses.size == 0):
                    poses = line
                else:
                    poses = np.vstack((poses,line))

    if ((poses.ndim==2) and (poses.shape[0]) != 0) and ((poses.shape[0]) != 0) and \
       ((cones.ndim==2) and (cones.shape[0]) != 0) and ((cones.shape[0]) != 0):
        
        #MELINDA: CHANGED THE CONES GRAPHING TO BE x,y, also changed the heading angle to reflect angle wrt y axis
        r = .5
        # changesx = r*np.cos(poses[:,2]) 
        # changesy = r*np.sin(poses[:,2]) 
        changesx = -r*np.sin(poses[:,2]) 
        changesy = r*np.cos(poses[:,2]) 
        # # function to add arrow on a graph
        
        scatter2 = plt.scatter(cones[:,0],cones[:,1], s=10, c='r', marker="o", label='landmark')
        for i in range(len(changesx)):
            scatter = plt.arrow(poses[:,0][i],poses[:,1][i],changesx[i],changesy[i],width=0.05)
        plt.pause(.1)
        scatter2.remove()
        plt.clf()
