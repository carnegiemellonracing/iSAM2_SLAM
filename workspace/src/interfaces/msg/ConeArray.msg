#Array of 2D cone locations (z = 0)

std_msgs/Header header

builtin_interfaces/Time orig_data_stamp

# White space and comments won't affect message files.
# Just make sure that the message files match 
# between publisher and subscriber in terms of 
# datatypes and the order the datatypes are listed.

geometry_msgs/Point[] blue_cones
geometry_msgs/Point[] yellow_cones
geometry_msgs/Point[] orange_cones
geometry_msgs/Point[] big_orange_cones
geometry_msgs/Point[] unknown_color_cones
