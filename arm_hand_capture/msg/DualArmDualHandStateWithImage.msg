# A message type combined with arm-hand state and the corresponding image frame.
# Note that message_filters can only work with messages that have Headers!!!

# Left arm and left hand
geometry_msgs/PoseStamped left_upperarm_pose
geometry_msgs/PoseStamped left_forearm_pose
geometry_msgs/PoseStamped left_hand_pose
arm_hand_capture/GloveState left_glove_state

# Right arm and right hand
geometry_msgs/PoseStamped right_upperarm_pose
geometry_msgs/PoseStamped right_forearm_pose
geometry_msgs/PoseStamped right_hand_pose
arm_hand_capture/GloveState right_glove_state

# Corresponding image
sensor_msgs/Image image
