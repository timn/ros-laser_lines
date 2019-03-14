# The visibilitiy history indicates the number of consecutive positive
# or negative sightings. If the history is negative, there have been
# as many negative sightings (object not visible) as the absolute
# value of the history. A positive value denotes as many positive
# sightings. 0 shall only be used during the initialization of the
# interface or if the visibility history is not updated.
int32                  visibility_history

# Line parameters, the point on line is optimized such that it is the
# point on the line with the smallest distance to the input frame
geometry_msgs/Point    point_on_line
geometry_msgs/Vector3  line_direction

# Direction towards the line, i.e. if the robot turns by this angle
# the robot will stand parallel to the line.
float32                bearing

# Extreme points of the line segment
geometry_msgs/Point point_1
geometry_msgs/Point point_2

# Length of line segment
float32     length

