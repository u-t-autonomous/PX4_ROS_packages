bool use_position				#send position target
bool use_speed					#send speed target
bool use_acceleration			#send acceleration target
bool use_yaw					#only check for position control !! Must be send in attitude control !!!
bool use_body_frame_offset		#this is for moving using the quad current heading as the reference
bool use_rate 					#for attctl or for yaw in posctl rate can be used instead of angle

float64 posZ_thrust
float64 posX_roll
float64 posY_pitch
float64 yaw

float64 velX
float64 velY
float64 velZ

float64 accX
float64 accY
float64 accZ