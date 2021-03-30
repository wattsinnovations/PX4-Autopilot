##########################################################################
#
# Propulsion System Info message which contains all of the (relevant) data
# contained in the m95040df SPI EEPROM.
#
# A PRISM propulsion ID module will listen to this data and then update
# PRISM status (Set params, reboot vehicle, and deny arming).
#
##########################################################################

uint64 timestamp			# Time since system start (microseconds)

uint8 propulsion_id
uint8 location
uint8 motor_index			# Watts motor index. Motor index and Location will need to match!
							# - FR = 1
							# - BR = 2
							# - BL = 3
							# - FL = 4

uint64 flight_time_ms		# Total accumulated flight time in milliseconds

# Location
uint8 LOCATION_FR = 1		# Front right
uint8 LOCATION_BR = 2		# Back right
uint8 LOCATION_BL = 3		# Back left
uint8 LOCATION_FL = 4		# Front left

# Propulsion ID
uint8 QUAD_U8ii = 1
uint8 COAXIAL_X8_U8ii = 2
