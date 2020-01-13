# Raw  LIDAR Stream.
#Header        header         # standard ROS message header
time stamp                    # packet timestamp
uint32 packetNumber           # packet number
uint32 packetSize
uint8[] data                  # packet contents 600 Block
