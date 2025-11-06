# Measured object with 3D position and real-world dimensions
# Result of multi-view triangulation

std_msgs/Header header

# Object class information
int32 class_id
string class_name
float32 confidence

# 3D position in base_link frame
geometry_msgs/Point position

# Real-world dimensions (meters)
geometry_msgs/Vector3 dimensions  # x=width, y=depth, z=height

# Depth from camera (meters)
float32 depth_from_camera

# Measurement quality metrics
float32 depth_error      # Estimated depth error (meters)
float32 size_error       # Estimated size error (meters)
int8 view_count          # Number of views used (2 or 3)
float32 matching_score   # Multi-view matching confidence (0-1)

# Debug information
float32 disparity                        # Pixel disparity between views
geometry_msgs/Point[] view_positions     # Camera positions for each view

# Bounding box dimensions (pixels) for cube detection
float32 bbox_width                       # Bounding box width in pixels
float32 bbox_height                      # Bounding box height in pixels
