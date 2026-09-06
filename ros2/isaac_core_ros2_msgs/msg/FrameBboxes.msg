# All detections for a single camera frame, as parallel arrays.
#
# Every array has the same length: one entry per detection. Index i refers to the same object
# in every array, so a consumer reads one detection as a slice across all of them:
#
#   for i in range(len(msg.target_name)):
#       print(msg.target_name[i], msg.x1[i], msg.lat[i])
#
# len(target_name) is the authoritative detection count. There is deliberately no `count`
# field: it would duplicate the array length and could disagree with it.
#
# Parallel arrays rather than a Bbox[] because Isaac Sim 6 cannot publish an array of nested
# messages -- its generic ROS 2 publisher exposes such a field as an unusable token[] that
# crashes when written, while primitive arrays publish correctly. Field names are unchanged
# from the 2023 Bbox message; only the index moves, from msg.bboxes[i].x1 to msg.x1[i].
#
# An all-empty message is meaningful: the frame was processed and nothing was found, which is
# different from no message at all.
#
# header.stamp is the simulation time of the frame the boxes were computed from, so a consumer
# can align detections with the matching image on /isaac_core/image_rgb rather than assuming
# the two streams arrive in lockstep.

std_msgs/Header header

# Name of each detected object's prim.
string[] target_name

# in_frame   - the object's projection falls inside the image bounds
# is_visible - in frame AND not occluded by other geometry
# Test is_visible for "can the camera actually see it".
bool[] in_frame
bool[] is_visible

# Bounding box edges in pixels: (x1, y1) top-left, (x2, y2) bottom-right.
# Only meaningful where in_frame is true.
int32[] x1
int32[] y1
int32[] x2
int32[] y2

# Geodetic position of each object. Degrees for lat/lon, metres above sea level for alt.
float32[] lat
float32[] lon
float32[] alt

# Object orientation in radians, euler XYZ.
float32[] roll
float32[] pitch
float32[] yaw

# Camera-to-object offset per world axis, in metres.
float32[] distance_x
float32[] distance_y
float32[] distance_z
