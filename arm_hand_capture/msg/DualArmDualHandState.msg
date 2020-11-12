# A representation describing the state of both hands and both arms, at the same time instance sync-ed by message_filters' Approximate Time Synchronization policy.

# Left arm and left hand
geometry_msgs/PoseStamped left_upperarm_pose
geometry_msgs/PoseStamped left_forearm_pose
geometry_msgs/PoseStamped left_hand_pose
# arm_hand_capture/GloveState left_glove_state

# Right arm and right hand
geometry_msgs/PoseStamped right_upperarm_pose
geometry_msgs/PoseStamped right_forearm_pose
geometry_msgs/PoseStamped right_hand_pose
arm_hand_capture/GloveState glove_state
