# 2D detection result from YOLO
# Represents a single detected object in image coordinates

std_msgs/Header header

# Object class information
int32 class_id
string class_name
float32 confidence

# Bounding box in pixel coordinates
# Format: [x1, y1, x2, y2] where (x1,y1) is top-left, (x2,y2) is bottom-right
float32[4] bbox

# Bounding box center in pixel coordinates
float32[2] bbox_center  # [u, v]

# Bounding box size in pixels
float32 bbox_width
float32 bbox_height

# Camera frame where the detection was made
string camera_frame
