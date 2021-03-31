##########################################################################
#
# Propulsion ID Info message. This message contains the arming status,
# configuration status, locations, and flight times for each Propulsion System.
#
##########################################################################

uint64 timestamp			# Time since system start (microseconds)

bool deny_arm				# Flag indicating if Propulsion ID is denying arming

bool[4] connected			# True if the EEPROM is connected

uint8[4] locations
uint8[4] propulsion_ids
uint64[4] flight_times
