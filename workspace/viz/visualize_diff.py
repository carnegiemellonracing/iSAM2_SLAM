import numpy as np
import matplotlib.pyplot as plt
import time
import copy

fig = plt.figure()
lm_step_colors = ['black', 'lightsteelblue', 'deepskyblue', 'blue', 'mediumpurple']
pose_step_colors=['lightcoral', 'lightsalmon', 'coral', 'red', 'crimson']

def parse_pose(line):
    result = line[1:len(line)-1]
    result = np.array(result.split(','))
    result = result.astype(float)
    return result

def parse_landmark(lines):
    '''
    Takes in the lines representing the landmark
    Returns the landmark as an np.array
    '''
    cur_landmark = np.array([])
    for line in lines:
        line = line.strip()
        if line == "]":
            break
        elif line == "[":
            continue
        elif cur_landmark.size == 0:
            x_coord = float(line[0:len(line) - 1])
            cur_landmark = np.append(cur_landmark, [x_coord])
        elif cur_landmark.size == 1:
            y_coord = float(line[0:len(line)])
            cur_landmark = np.append(cur_landmark, [y_coord])

    return cur_landmark

num_steps = 7
for j in range(num_steps, -1, -1): # printing in reverse order to show progression
# for j in range(num_steps):
    filename = "src/isam2/data/estimate_t" + str(j) + ".txt"
    with open(filename) as f:
        lines = f.readlines() # list containing lines of file

    # Don't do anything if the file is empty
    if len(lines) == 0:
        continue

    landmarks = np.array([])
    poses = np.array([])

    pose = 0
    landmark = 0
    for (line_idx,line) in enumerate(lines):
        if line_idx == 0 or line_idx == 1: # lines 0 and 1 are bogus
            continue

        print(line)
        line = line.strip() # remove leading/trailing white spaces
        if line:
            if(line[0:7]== "Value x"): # indicate next line is pose
                pose = 1
            elif(line[0:7]== "Value l"): # indicate next lines are landmark
                landmark = 1
            elif pose == 1:
                cur_pose = parse_pose(line)

                if(poses.size == 0):
                    poses = cur_pose
                else:
                    poses = np.vstack((poses,cur_pose))

                pose = 0

            elif landmark == 1:
                cur_landmark = parse_landmark(lines[line_idx:line_idx+5])

                if(landmarks.size == 0):
                    landmarks = cur_landmark
                else:
                    landmarks = np.vstack((landmarks, cur_landmark))

                landmark = 0

            else:
                continue



    if (((poses.shape[0]) != 0) and ((landmarks.shape[0]) != 0)
            and (poses.ndim==2 and landmarks.ndim==2)):
        scatter = plt.scatter(poses[:,0], poses[:,1], s=10,
                    c=pose_step_colors[j % len(pose_step_colors)], marker="x", label='pose')
        scatter2 = plt.scatter(landmarks[:,0],landmarks[:,1],s=10,
                    c=lm_step_colors[j % len(lm_step_colors)], marker="o", label='landmark')
        print("plotting")
plt.show()
