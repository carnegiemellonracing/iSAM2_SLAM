from pathlib import Path
from rosbags.rosbag2 import Writer, Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.interfaces import ConnectionExtRosbag2
import os
import shutil
from typing import TYPE_CHECKING, cast

input_bag = '/home/dale/rosbags/fourth-run-mock-demo/'
output_bag = '/home/dale/rosbags/fourth-run-mock-demo-stamped/'

# How were these calculated: ros2 topic echo /lidar_points and
# /filter/velocity and then end the rosbag and analyze the timestamps
delta_sec = 120587229
delta_nanosec = 285524633

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




def modify_timestamps():
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

            if connection.topic == '/lidar_points':
                orig_sec = msg.header.stamp.sec
                orig_nanosec = msg.header.stamp.nanosec
                # print(orig_sec, orig_nanosec)
                new_sec = orig_sec + delta_sec
                new_nanosec = orig_nanosec + delta_nanosec
                msg.header.stamp.sec = new_sec
                msg.header.stamp.nanosec = new_nanosec
                writer.write(connection_map[connection.id], timestamp,
                            typestore.serialize_cdr(msg, connection.msgtype))
            else:
                writer.write(connection_map[connection.id], timestamp,
                            typestore.serialize_cdr(msg, connection.msgtype))
        print("Finished creating rosbag")


typestore = get_typestore(Stores.LATEST)
bag_info()
modify_timestamps()
