import numpy as np
import matplotlib.pyplot as plt

CHUNKING_DATA_FILE = "../src/isam2/data/chunking_data.txt"


"""
Defining the format the chunking data should be stored in chunking_data.txt

Separators: Chunks are separated by `Chunk N` where N is the chunk ID
Contents: Chunks have blue and yellow cones. 
Blue cones are represented as `Blue: float, float`
Yellow cones are represented as `Yellow: float, float`
Chunks end with `End of chunk`

Chunks are separated from start of the next chunk with a newline
"""

CHUNK_END_INDIC = "End of chunk"
BLUE_CONE_INDIC = "Blue"
YELLOW_CONE_INDIC = "Yellow"

BLUE_COLOR_ONE = 'r'
BLUE_COLOR_TWO = 'b'
YELLOW_COLOR_ONE = 'r'
YELLOW_COLOR_TWO = 'b'
plt.show()
def process_color(line, indicator, x_array, y_array):
    """
    @param indicator A string that indicates what information the line holds
    @param x_array An np array for the x coordinate (pass by reference)
    @param y_array An np array for the y coordinate (pass by reference)
    """
    comma_idx = line.index(',')
    cone_x = float(line[len(indicator) + 1:comma_idx])
    cone_y = float(line[comma_idx+1:])
    x_array = np.append(x_array, cone_x)
    y_array = np.append(y_array, cone_y)
    return (x_array, y_array)

def visualize_chunks(all_chunks):
    """
    @param all_chunks A list of 4-tuples. Each tuple contains 4 np arrays
    holding information about the blue x and y cones making up a chunk
    and the yellow x and y cones making up a chunk. Coordinate indices
    correspond with one another. 
    """
    blue_color = BLUE_COLOR_ONE
    yellow_color = YELLOW_COLOR_ONE
    idx = 1
    print("VISUALIZING")
    print(all_chunks)
    for chunk in all_chunks:
        blue_cones_x, blue_cones_y, yellow_cones_x, yellow_cones_y = chunk
        print(idx)
        idx += 1
        plt.scatter(blue_cones_x, blue_cones_y, s=10, c=blue_color, marker='x', label=f"blue{idx}")
        plt.scatter(yellow_cones_x, yellow_cones_y, s=10, c=yellow_color, marker='x', label=f"yellow{idx}")
        plt.pause(1)
        if blue_color == BLUE_COLOR_ONE:
            blue_color = BLUE_COLOR_TWO
            yellow_color = YELLOW_COLOR_TWO
        else:
            blue_color = BLUE_COLOR_ONE
            yellow_color = YELLOW_COLOR_ONE
    plt.show(block=True)

def visualize_main():
    run = True
    with open (CHUNKING_DATA_FILE) as f:
        lines = f.readlines()
        if len(lines) == 0: 
            return
        
        # Represents the index of the line 
        idx = 0
        all_chunks = [] # This is a list of np array tuples
        cur_chunk_blue_cones_x = np.array([])
        cur_chunk_blue_cones_y = np.array([])
        cur_chunk_yellow_cones_x = np.array([])
        cur_chunk_yellow_cones_y = np.array([])

        while idx < len(lines):
            line = lines[idx]
            line = line.strip()
            # Check the indicator on the line 
            
            if CHUNK_END_INDIC in line:
                all_chunks.append((cur_chunk_blue_cones_x, cur_chunk_blue_cones_y, 
                                cur_chunk_yellow_cones_x, cur_chunk_yellow_cones_y))

                cur_chunk_blue_cones_x = np.array([])
                cur_chunk_blue_cones_y = np.array([])
                cur_chunk_yellow_cones_x = np.array([])
                cur_chunk_yellow_cones_y = np.array([])
            elif BLUE_CONE_INDIC in line: 
                cur_chunk_blue_cones_x, cur_chunk_blue_cones_y = process_color(line, BLUE_CONE_INDIC, cur_chunk_blue_cones_x, cur_chunk_blue_cones_y)
                print(BLUE_CONE_INDIC, cur_chunk_blue_cones_x, cur_chunk_blue_cones_y)
            elif YELLOW_CONE_INDIC in line:
                cur_chunk_yellow_cones_x, cur_chunk_yellow_cones_y = process_color(line, YELLOW_CONE_INDIC, cur_chunk_yellow_cones_x, cur_chunk_yellow_cones_y)
                print(YELLOW_CONE_INDIC, cur_chunk_yellow_cones_x, cur_chunk_yellow_cones_y)
            
            idx += 1
        print("visualizing")
        visualize_chunks(all_chunks)

visualize_main()