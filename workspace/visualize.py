import numpy as np
import matplotlib.pyplot as plt

with open('src/isam2/data/estimate2.txt') as f:
    lines = f.readlines() # list containing lines of file

    landmarks = np.array([])
    poses = np.array([])

    i = 1
    pose = 0
    landmark = 0
    for line in lines:
        line = line.strip() # remove leading/trailing white spaces
        # print(line)
        if line:
            if i == 1 or i==2:
                i = i + 1
            else:
                if(line[0:7]== "Value x"):
                    pose = 1
                    #next line is pose
                elif(line[0:7]== "Value l"):
                    landmark = 1
                    #next line is landmark
                elif pose == 1:
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
                elif landmark > 0:
                    if landmark == 1:
                        tempLandmark = []
                    elif landmark == 2:
                        line = line[0:-1] #strip ;
                        tempLandmark.append(line)
                    elif landmark == 3:
                        tempLandmark.append(line)
                    elif landmark == 4:
                        tempLandmark = np.array(tempLandmark)
                        tempLandmark = tempLandmark.astype(float)
                        if(landmarks.size == 0):
                            landmarks = tempLandmark
                        else:
                            landmarks = np.vstack((landmarks,tempLandmark))        
                    landmark = (landmark+1)%5
                        
# print("poses:",poses)
# print("landmark:",landmarks)

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.scatter(poses[:,0], poses[:,1], s=10, c='b', marker="x", label='pose')
ax1.scatter(landmarks[:,0],landmarks[:,1], s=10, c='r', marker="o", label='landmark')
plt.legend(loc='upper left')
plt.show()