import numpy as np
import matplotlib.pyplot as plt
import time
import copy
import pdb
import os

# Constants
BLUE_LANDMARK_INDIC = "Value b"
YELLOW_LANDMARK_INDIC = "Value y"
POSE_INDIC = "Value x"
CHUNK_FILE = "../src/isam2/data/chunks.txt"

fig = plt.figure()
plt.ion()
#plt.legend(loc='upper left')
plt.show()
run = True

def load_chunks():
    chunks = []
    cone_map = {'blue': {}, 'yellow': {}}
    try:
        if os.path.exists(CHUNK_FILE):
            with open(CHUNK_FILE) as f:
                for line in f:
                    if line.startswith("CHUNK:"):
                        parts = line.strip().split(':')[1].split(',')
                        chunks.append((float(parts[0]), float(parts[1])))
                    elif line.startswith("BLUE:"):
                        _, cone_id, chunk_id = line.strip().split(':')
                        cone_map['blue'][int(cone_id)] = int(chunk_id)
                    elif line.startswith("YELLOW:"):
                        _, cone_id, chunk_id = line.strip().split(':')
                        cone_map['yellow'][int(cone_id)] = int(chunk_id)
    except Exception as e:
        pass
    return chunks, cone_map

def darken_color(color, factor=0.7):
    r, g, b, a = color
    return (r * factor, g * factor, b * factor, a)


while run:
    chunks, cone_map = load_chunks()

    # with open('../src/isam2/saved_data/controls_sim_colored_cones_track.txt') as f:
    with open("../src/isam2/data/current_estimates.txt") as f:
        lines = f.readlines() # list containing lines of file
        # Don't do anything if the file is empty
        if len(lines) == 0:
            continue

        blue_landmarks_x = np.array([])
        blue_landmarks_y = np.array([])
        yellow_landmarks_x = np.array([])
        yellow_landmarks_y = np.array([])
        pose_x = np.array([])
        pose_y = np.array([])


        i = 1
        pose = 0
        landmark = 0
        # We ignore the first 2 lines
        idx = 2
        while (idx < len(lines)):
            """
            Strategy:
            1.) Case on the indicator
            2.) Process the data (according to the indicator) appropriately
            - Blue: x and y value that spans 5 lines; x on line +2; y on line +3 (wrt indicator line)
            - Yellow: x and y value that spans 5 lines; x on line +2; y on line +3 (wrt indicator line)
            - Pose: contains x, y, theta info; spans 3 lines: data on line +1 (wrt indicator line)
            """
            line = lines[idx]
            line = line.strip()
            indic = line[0:7]
            if indic == BLUE_LANDMARK_INDIC:
                first_colon_idx = line.find(':')
                second_colon_idx = line.find(':', first_colon_idx + 1)
                x = (float) (line[first_colon_idx + 1:second_colon_idx])
                y = (float) (line[second_colon_idx + 1:])

                blue_landmarks_x = np.append(blue_landmarks_x, x)
                blue_landmarks_y = np.append(blue_landmarks_y, y)


            elif indic == YELLOW_LANDMARK_INDIC:
                first_colon_idx = line.find(':')
                second_colon_idx = line.find(':', first_colon_idx + 1)
                x = (float) (line[first_colon_idx + 1:second_colon_idx])
                y = (float) (line[second_colon_idx + 1:])

                yellow_landmarks_x = np.append(yellow_landmarks_x, x)
                yellow_landmarks_y = np.append(yellow_landmarks_y, y)


            elif indic == POSE_INDIC:
                first_colon_idx = line.find(':')
                second_colon_idx = line.find(':', first_colon_idx + 1)
                x = (float) (line[first_colon_idx + 1:second_colon_idx])
                y = (float) (line[second_colon_idx + 1:])

                pose_x = np.append(pose_x, x)
                pose_y = np.append(pose_y, y)

            idx += 1

        if (len(pose_x) > 0 ):
            print("finished reading")

            # Create color arrays based on chunk membership
            blue_colors = []
            for i in range(len(blue_landmarks_x)):
                chunk_id = cone_map['blue'].get(i, -1)
                if chunk_id != -1:
                    # Alternate colors based on chunk ID
                    blue_colors.append('red' if chunk_id % 2 == 0 else 'orange')
                else:
                    blue_colors.append('b')  # Default blue for unchunked

            yellow_colors = []
            for i in range(len(yellow_landmarks_x)):
                chunk_id = cone_map['yellow'].get(i, -1)
                if chunk_id != -1:
                    yellow_colors.append('red' if chunk_id % 2 == 0 else 'orange')
                else:
                    yellow_colors.append('y')  # Default yellow for unchunked

            # Plot with chunk colors
            scatter = plt.scatter(pose_x, pose_y, s=10, c='r', marker="x", label='pose')
            scatter2 = plt.scatter(blue_landmarks_x, blue_landmarks_y, s=10, c=blue_colors, marker="o", label='blue landmarks')
            scatter3 = plt.scatter(yellow_landmarks_x, yellow_landmarks_y, s=11, c=yellow_colors, marker="o", label='yellow landmarks')
            
            # Display ID labels
            # for i, (x, y) in enumerate(zip(blue_landmarks_x, blue_landmarks_y)):
            #     plt.text(x + 0.2, y + 0.2, str(i), color='black', fontsize=8)
            # for i, (x, y) in enumerate(zip(yellow_landmarks_x, yellow_landmarks_y)):
            #     plt.text(x + 0.2, y + 0.2, str(i), color='black', fontsize=8)

            print("plotting")
            plt.pause(1)
            scatter.remove()
            scatter2.remove()
            scatter3.remove()









