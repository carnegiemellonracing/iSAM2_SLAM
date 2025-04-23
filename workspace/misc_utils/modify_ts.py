"""
How to use:
1.) Make sure the input bag and the
output bag have been specified

2.) Calculate the delta between the
lidar timestamps and the odometry
timestamps
- This can be done by ros2 topic echo
  and comparing the timesteps
"""

from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.interfaces import ConnectionExtRosbag2
import os
import shutil
from typing import TYPE_CHECKING, cast
import sys


typestore = get_typestore(Stores.LATEST)
custom_msg_paths = [
        'interfaces/msg/ActuatorsInfo.msg',
        'interfaces/msg/ConeArray.msg',
        'interfaces/msg/ControllerInfo.msg',
        'interfaces/msg/ControlsState.msg',
        'interfaces/msg/ControlAction.msg',
        'interfaces/msg/SplineFrames.msg']

def bag_info():
    # Create a typestore and get the string class.

    # need to register custom types
    add_types = {}
    for p in custom_msg_paths:
        msg_text = Path(p).read_text()
        add_types.update(get_types_from_msg(msg_text, p[0:-4]))
    print(add_types)
    typestore.register(add_types)


def parse_arguments():
    assert(len(sys.argv) == 6)
    # How were these calculated: ros2 topic echo /lidar_points and
    # /filter/velocity and then end the rosbag and analyze the timestamps
    # This is for fourth-run-mock-demo
    # delta_sec = 120587229
    # delta_nanosec = 285524633
    #
    # This is for third-run-demo-practice-five-laps
    delta_sec = sys.argv[1]
    delta_nanosec = sys.argv[2]

    # Used for trimming the rosbag
    # Times for fourth run stamped
    # start_trim = 1714592825
    # Personal notes 1714592862 is the start of the second lap
    # end_trim = 1714592899
    #
    # Times for third run stamped
    # start_trim = 1714503088 # The car hasn't started moving yet
    start_trim = sys.argv[3]
    end_trim = sys.argv[4]


    prefix = "./"
    input_bag = sys.argv[5]
    output_bag = sys.argv[6]

    return (delta_sec, delta_nanosec), (start_trim, end_trim), (input_bag, output_bag)




def modify_timestamps():
    (delta_sec, delta_nanosec), (start_trim, end_trim), (input_bag, output_bag) = parse_arguments()


    if os.path.exists(output_bag):
        print("\n\nIssue: the output directory", output_bag, "already exists\n")
        shutil.rmtree(output_bag)
    with Reader(input_bag) as reader, Writer(output_bag) as writer:

        connection_map = {}

        for c in reader.connections:
            connection_map[c.id] = writer.add_connection(c.topic,
                                                        c.msgtype,
                                                        typestore=typestore)

        for connection, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            # If the topic is /lidar_points, add the delta
            if connection.topic == '/lidar_points':
                orig_sec = msg.header.stamp.sec
                orig_nanosec = msg.header.stamp.nanosec
                # print(orig_sec, orig_nanosec)
                new_sec = orig_sec + delta_sec
                new_nanosec = orig_nanosec + delta_nanosec

                if new_sec >= start_trim and new_sec <= end_trim:
                    msg.header.stamp.sec = new_sec
                    msg.header.stamp.nanosec = new_nanosec
                    writer.write(connection_map[connection.id], timestamp,
                            typestore.serialize_cdr(msg, connection.msgtype))
            else:
                orig_sec = timestamp / 1e9
                print("Timestamp seconds of non /lidar_points message", orig_sec)

                # This is the trimming logic
                if orig_sec >= start_trim and orig_sec <= end_trim:
                    writer.write(connection_map[connection.id], timestamp,
                            typestore.serialize_cdr(msg, connection.msgtype))
        print("Finished creating rosbag")


typestore = get_typestore(Stores.LATEST)
bag_info()
modify_timestamps()
