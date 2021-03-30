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

bool missing				# An EEPROM is missing
uint8 missing_index			# Indicated which EEPROM is missing
							# - Indexed from 1, when missing is false this shall be value 0
