# Single scan from an array of sonar sensors

Header header            # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

float32 arc_range        # the size of the arc that the reading is valid for [rad]
float32 min_range        # the minimum range value [m]
float32 max_range        # the maximum range value [m]

float32[] ranges         # range data [m]
float32[] angles         # base angle corresponding to each entry in ranges [rad]
