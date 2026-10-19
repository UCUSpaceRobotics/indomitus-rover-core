# Dataset capture state of the rover's cameras (rover_cameras).
#
# Published latched by every camera_node on cameras/<name>/dataset_state and,
# aggregated over all cameras, by dataset_node on cameras/dataset_state. The
# recording flag lives in the cameras; this is a read-back, never a command.

# At least one camera is saving a frame every record_interval seconds.
bool recording

# Cameras currently capturing frames (0 or 1 on a single camera's topic).
uint32 cameras_live

# Frames written to disk since the camera node(s) started.
uint32 saved
