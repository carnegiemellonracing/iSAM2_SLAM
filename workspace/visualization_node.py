import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import time

# just creates the list of random coords to turn into the landmarks array
landmarks_list = [[round(np.random.uniform(-50,50), 5), round(np.random.uniform(-50,50), 5)] for i in range(100)]
landmarks = np.array(landmarks_list)
# print(landmarks)

# Creates landmarks and poses arrays (poses not relevant)
# Questions: What will the ground truth cone data look like?
# with open("squirrel.txt") as f:
#         lines = f.readlines() # list containing lines of file
#     # Don't do anything if the file is empty
#     if len(lines) == 0:
#         print("squirrel")
#         continue
#     landmarks = np.array([])
#     poses = np.array([])
#     i = 1
#     pose = 0
#     landmark = 0
#     for line in lines:
#         line = line.strip() # remove leading/trailing white spaces
#         if line:
#             if i == 1 or i==2:
#                 i = i + 1
#             else:
#                 if(line[0:7]== "Value x"):
#                     pose = 1
#                     #next line is pose
#                 elif(line[0:7]== "Value l"):
#                     # pass
#                     landmark = 1
#                     #next line is landmark
#                 elif pose == 1:
#                     # Create array for poses and update it incrementally
#                     #single line in pose
#                     #strip parentheses
#                     temp = line[1:len(line)-1]
#                     temp = np.array(temp.split(','))
#                     temp = temp.astype(float)
#                     if(poses.size == 0):
#                         poses = temp

#                     else:
#                         poses = np.vstack((poses,temp))
#                     pose = 0
#                 elif landmark == 1:
#                     # pass
#                     temp = line[1:len(line)-1]
#                     temp = np.array(temp.split(','))
#                     temp = temp.astype(float)
#                     if(landmarks.size == 0):
#                         landmarks = temp
#                     else:
#                         landmarks = np.vstack((landmarks,temp))
#                     landmark = 0

# create points array by applying random offset to landmarks array
offsets = np.random.uniform(-5, 5, np.shape(landmarks))
points = landmarks + offsets

def plot_five(points, landmarks, clr):
    # Read 5 coords from points array
    five_points = points[:5]
    # slice the array after we've saved first 5 coords
    # create np array of x coords and y coords for scatter plot
    points_x = five_points[:, 0]
    print(points_x)
    points_y = five_points[:, 1]
    # plot the points
    scatter = plt.scatter(points_x, points_y, color= clr)
    # Create KD-tree from landmarks for efficient nearest neighbor search
    landmark_tree = cKDTree(landmarks)
    # Query the nearest neighbors for all points in five_points
    distances, indices = landmark_tree.query(five_points, k=1)
    # Retrieve the nearest neighbor coordinates
    landmark_neighbors = landmarks[indices]
    # Plot nearest neighbor points
    plt.scatter(landmark_neighbors[:, 0], landmark_neighbors[:, 1], color='red')
    # Plot lines between corresponding points
    for i in range(len(five_points)):
        plt.plot([five_points[i, 0], landmark_neighbors[i, 0]], [five_points[i, 1], landmark_neighbors[i, 1]], color='red', linestyle='solid')

    plt.pause(0.1)
    return points[5:]

colors_list = ['orange', 'yellow', 'green', 'blue', 'purple']

fig = plt.figure()
plt.ion()
plt.show()

color_idx = 0;
while True:
    plot_color = colors_list[color_idx % 5]
    # print(plot_color)
    color_idx += 1
    points = plot_five(points, landmarks, plot_color)



