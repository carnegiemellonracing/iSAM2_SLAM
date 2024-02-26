import numpy as np
import matplotlib.pyplot as plt
import time

fig = plt.figure()
plt.ion()
#plt.legend(loc='upper left')
plt.show()

while True:
    # with open('src/isam2/data/estimate2.txt') as f:
    with open("squirrel.txt") as f:
        lines = f.readlines() # list containing lines of file

    # Don't do anything if the file is empty
    if len(lines) == 0:
        continue

    landmarks = np.array([])
    poses = np.array([])

    i = 1
    pose = 0
    landmark = 0
    for line in lines:
        line = line.strip() # remove leading/trailing white spaces
        if line:
            if i == 1 or i==2:
                i = i + 1
            else:
                if(line[0:7]== "Value x"):
                    pose = 1
                    #next line is pose
                elif(line[0:7]== "Value l"):
                    # pass
                    landmark = 1
                    #next line is landmark
                elif pose == 1:
                    # Create array for poses and update it incrementally
                    #single line in pose
                    #strip parentheses
                    temp = line[1:len(line)-1]
                    temp = np.array(temp.split(','))
                    temp = temp.astype(float)
                    if(poses.size == 0):
                        poses = temp
                    else:
                        poses = np.vstack((poses,temp))
                    pose = 0
                elif landmark == 1:
                    # pass
                    temp = line[1:len(line)-1]
                    temp = np.array(temp.split(','))
                    temp = temp.astype(float)
                    if(landmarks.size == 0):
                        landmarks = temp
                    else:
                        landmarks = np.vstack((landmarks,temp))
                    landmark = 0
    # if ((poses.shape[0]) != 0) and ((poses.shape[0]) != 0) and (poses.ndim==2 and landmarks.ndim==2):
    if ((poses.shape[0]) != 0) and ((poses.shape[0]) != 0) and (poses.ndim==2):
        r = .2
        changesx = r*np.cos(poses[:,2]) 
        changesy = r*np.sin(poses[:,2]) 
        # # function to add arrow on a graph
        for i in range(len(changesx)):
            scatter = plt.arrow(poses[:,0][i],poses[:,1][i],changesx[i],changesy[i],width=0.05)

        # scatter2 = plt.scatter(landmarks[:,0],landmarks[:,1], s=10, c='r', marker="o", label='landmark')
        plt.pause(.1)
        # scatter2.remove()

        # fig.clf()
        # scatter2.remove()