# State of a single zone monitoring slot as reported by the sensor.
#
# A zone monitoring packet always carries 16 slots. Slots that are not
# associated with a live zone have `live` set to false, the remaining fields of
# such a slot carry no meaning.

# trigger_type values (the operation mode configured for the zone)
uint8 TRIGGER_TYPE_NONE=0       # no mode specified
uint8 TRIGGER_TYPE_OCCUPANCY=1  # zone triggers when an object enters it
uint8 TRIGGER_TYPE_VACANCY=2    # zone triggers when an object leaves it

# trigger_status values
uint8 TRIGGER_STATUS_DEASSERTED=0
uint8 TRIGGER_STATUS_ASSERTED=1

bool live                # true for all live zones, regardless of the alert
                         # trigger state (empty slots are false).
uint8 id                 # index of the zone within the sensor zone set, only
                         # meaningful for a live zone.
string label             # label of the zone as defined by the zone set stored
                         # on the sensor, empty when unavailable.
uint8 error_flags        # error flags: missing data, low confidence pixels,
                         # zone obscured.
uint8 trigger_type       # zone operation mode, see TRIGGER_TYPE_* above.
uint8 trigger_status     # zone alert state, see TRIGGER_STATUS_* above.
uint32 triggered_frames  # saturating count of frames triggered consecutively,
                         # resets on deassertion.
uint32 count             # saturating latest count of points in the zone,
                         # updates live regardless of trigger status.
uint32 occlusion_count   # count of points below minimum range in the zone.
uint32 invalid_count     # count of invalid points in the zone.
uint32 max_count         # maximum count of points that can be detected in the
                         # zone.
uint32 min_range         # minimum range of points in the zone in millimeters.
uint32 max_range         # maximum range of points in the zone in millimeters.
uint32 mean_range        # mean range of points in the zone in millimeters.
