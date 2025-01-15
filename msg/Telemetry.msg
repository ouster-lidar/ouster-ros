# This message defines the telemetry data for Ouster sensors.
# Message header
std_msgs/Header header
# Telemetry fields
uint16 countdown_thermal_shutdown # the thermal shutdown countdown.
uint16 countdown_shot_limiting    # the shot limiting countdown.
uint8 thermal_shutdown            # the thermal shutdown status
uint8 shot_limiting               # the shot limiting status
