# This message defines the telemetry data for Ouster sensors.
# Message header
std_msgs/Header header
# Telemetry fields for more information on these fields and their meaning, please review:
# https://static.ouster.dev/sensor-docs/image_route1/image_route2/thermal_int_guide/therm_int_guide.html
uint16 countdown_thermal_shutdown # the thermal shutdown countdown.
uint16 countdown_shot_limiting    # the shot limiting countdown.
uint8 thermal_shutdown            # the thermal shutdown status. 0 = NORMAL, 1 = SHUTDOWN IMMINENT.
uint8 shot_limiting               # the shot limiting status. 0 = NORMAL OPERATION.
