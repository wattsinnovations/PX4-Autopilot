##########################################################################
#
# Propulsion ID Info message. This message contains the arming status,
# configuration status, locations, and flight times for each Propulsion System.
#
##########################################################################

uint64 timestamp			# Time since system start (microseconds)

bool deny_arm				# Flag indicating if Propulsion ID is denying arming

bool mismatch				# Flag indicating if there is a Propulsion System mismatch
							# - Locations must be mutually exclusive
							# - Propulsion IDs must match

bool[4] connected			# True if the EEPROM is connected
