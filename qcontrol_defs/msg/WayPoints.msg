#This message defines the list of constrained points and yaw that should be use to
#generate the trajectory.

ConstraintAxis[] yaw			#Can be empy if no need of a trajectory for the YAW. ELSE it should have
								#the same length as time.
ConstraintAxis[] x				#Should have the same length as time
ConstraintAxis[] y
ConstraintAxis[] z

float64[] t						#Can be a list of two if you have just the initial and final time. Optimal segment time will be used instead
float64[] corridors