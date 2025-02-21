import numpy as np
import matplotlib.pyplot as plt
import time
import copy
import pdb

fig = plt.figure()
plt.ion()
#plt.legend(loc='upper left')
plt.style.use("dark_background")

def display_lap(filename, cone_color):
    # with open('src/isam2/data/estimate2.txt') as f:
    with open(filename) as f:
        lines = f.readlines() # list containing lines of file
    # Don't do anything if the file is empty
    if len(lines) == 0:
        return
    landmarks = np.array([])
    poses = np.array([])

    cur_landmark =np.array([])
    i = 1
    pose = 0
    landmark = 0

    # Read through the file
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
                        poses = np.array([temp])
                    else:
                        poses = np.vstack((poses,temp))
                    pose = 0
                elif landmark == 1:
                    # pass
                    # temp = line[1:len(line)-1]
                    # temp = np.array(temp.split(','))
                    # temp = temp.astype(float)

                    # Case on the line:
                    if line == "]":
                        if(landmarks.size == 0):
                            landmarks = np.array([cur_landmark])
                            cur_landmark = np.array([])
                        else:
                            landmarks = np.vstack((landmarks, cur_landmark))
                            cur_landmark = np.array([])

                        landmark = 0
                    elif line == "[":
                        continue
                    elif cur_landmark.size == 0:
                        x_coord = float(line[0:len(line) - 1])
                        cur_landmark = np.append(cur_landmark, [x_coord])
                    elif cur_landmark.size == 1:
                        y_coord = float(line[0:len(line)])
                        cur_landmark = np.append(cur_landmark, [y_coord])

    # if ((poses.shape[0]) != 0) and ((poses.shape[0]) != 0) and (poses.ndim==2):
    if ((poses.shape[0]) != 0) and ((landmarks.shape[0]) != 0) and (poses.ndim==2 and landmarks.ndim==2):
        plt.scatter(poses[:,0], poses[:,1], s=10, c='b', marker="x", label='pose')
        plt.scatter(landmarks[:,0],landmarks[:,1], s=10, c=cone_color, marker="o", label='landmark')
        print("plotting")
        plt.pause(1)
        

display_lap("../src/isam2/saved_data/lap1_estimates.txt", 'g')
display_lap("../src/isam2/saved_data/lap2_estimates.txt", 'r')
plt.show(block=True)