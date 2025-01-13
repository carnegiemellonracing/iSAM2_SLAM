import numpy as np
import math
import matplotlib.pyplot as plt

init_offset_lon = 0
init_offset_lat = 0

def degrees_to_radians(deg):
    return float(deg) * 3.14159265358979 / 180.0

def lat_lon_to_meters(cur_cone):
    '''
    converts latitude longitude degrees to meters

    cur_cone[0] = longitude
    cur_cone[1] = latitude
    '''

    lon = cur_cone[1]
    lat = cur_cone[0]
    lon -= init_offset_lon
    lat -= init_offset_lat

    LON_DEG_TO_METERS = 111319.5 * math.cos(degrees_to_radians(lon))

    LAT_DEG_TO_METERS = 111111
    lat *= LAT_DEG_TO_METERS
    lon *= LON_DEG_TO_METERS

    cur_cone[1] = lon
    cur_cone[0] = lat

def plot_data(blue_cones, yellow_cones):
    '''
    color is a char: 'b' or 'c'
    '''
    print(init_offset_lat, init_offset_lon)
    start_cones = np.array([blue_cones[0, :], yellow_cones[0, :]])
    print(init_offset_lat, init_offset_lon)
    print(start_cones)
    plt.scatter(blue_cones[1:, 1], blue_cones[1:, 0], s=10, c='blue', marker='x', label='cone')
    plt.scatter(yellow_cones[1:, 1], yellow_cones[1:, 0], s=10, c='orange', marker='x', label='cone')
    plt.scatter(start_cones[:, 1], start_cones[:, 0], s=10, c='red', marker='x', label='cone')
    plt.show()


def read_file():
    with open("cone_pos.txt") as f:
        lines = f.readlines()

    read_first_x = False
    read_first_y = False

    all_cones = np.array([])
    cur_cone = np.array([])

    blue_cones = np.array([])
    yellow_cones = np.array([])

    '''
    Information about cone_pos.txt
    First blue cones were read in counter clockwise order, starting at the orange cone
    Second, yellow cones were read, again in counter clockwise order
    '''
    status = ""
    for line in lines:
        line = line.strip()
        print(status)
        print(line)

        if len(line) == 0:
            lat_lon_to_meters(cur_cone)
            if all_cones.size == 0:
                all_cones = cur_cone
            else:
                all_cones = np.vstack((all_cones, cur_cone))
            cur_cone = np.array([])
            continue

        elif line[0] == "x":
            lat = float(line[line.index(":") + 1:])
            cur_cone = np.append(cur_cone, lat)

            if not read_first_x:
                read_first_x = True
                global init_offset_lat
                init_offset_lat = lat

        elif line[0] == "y":
            lon = float(line[line.index(":") + 1:])
            cur_cone = np.append(cur_cone, lon)

            if not read_first_y:
                read_first_y = True
                global init_offset_lon
                init_offset_lon = lon

        elif line == "BLUE":
            status = "READING BLUE"
            continue
        elif line == "YELLOW":
            status = "READING YELLOW"
            blue_cones = all_cones
            all_cones = np.array([])
    yellow_cones = all_cones
    print("<---- Printing init offset ---->")
    print(init_offset_lat, init_offset_lon)
    plot_data(blue_cones, yellow_cones)

read_file()
