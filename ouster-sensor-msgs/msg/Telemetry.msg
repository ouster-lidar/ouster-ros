# This message defines the telemetry data for Ouster sensors.

# ThermalShutdownStatus thermal_shutdown
uint8 THERMAL_SHUTDOWN_NORMAL=0    # Normal operation
uint8 THERMAL_SHUTDOWN_IMMINENT=1  # Thermal Shutdown imminent

# ShotLimitingStatus shot_limiting
uint8 SHOT_LIMITING_NORMAL=0          # Normal operation
uint8 SHOT_LIMITING_IMMINENT=1        # Shot limiting imminent
uint8 SHOT_LIMITING_REDUCTION_0_10=2  # Shot limiting reduction by 0 to 10%
uint8 SHOT_LIMITING_REDUCTION_10_20=3 # Shot limiting reduction by 10 to 20%
uint8 SHOT_LIMITING_REDUCTION_20_30=4 # Shot limiting reduction by 20 to 30%
uint8 SHOT_LIMITING_REDUCTION_30_40=5 # Shot limiting reduction by 30 to 40%
uint8 SHOT_LIMITING_REDUCTION_40_50=6 # Shot limiting reduction by 40 to 50%
uint8 SHOT_LIMITING_REDUCTION_50_60=7 # Shot limiting reduction by 50 to 60%
uint8 SHOT_LIMITING_REDUCTION_60_70=8 # Shot limiting reduction by 60 to 70%
uint8 SHOT_LIMITING_REDUCTION_70_75=9 # Shot limiting reduction by 70 to 80%

# Message header
std_msgs/Header header
# Telemetry fields for more information on these fields and their meaning, please review:
# https://static.ouster.dev/sensor-docs/image_route1/image_route2/thermal_int_guide/therm_int_guide.html
uint16 countdown_thermal_shutdown # the thermal shutdown countdown.
uint16 countdown_shot_limiting    # the shot limiting countdown.
uint8 thermal_shutdown            # the thermal shutdown status. 0 = NORMAL, 1 = SHUTDOWN IMMINENT.
uint8 shot_limiting               # the shot limiting status. 0 = NORMAL OPERATION.