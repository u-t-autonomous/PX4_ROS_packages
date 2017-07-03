#A constrained Value represents a constraint on x,y,z, yaw derivatives.
#For example the constraint equality : Vx = 2.0 or Vx <= 2.0 can be write with the following ConstraintValue Object :
### derive_order = FIRST_ORDER
### value = 2.0
### comparison = EQUAL or comparison = INF
# For yaw we currently support only first order constraint. And second order constraints 
#for the position. 

uint8 EQUAL = 1
uint8 INF = 2
uint8 SUP = 3

uint8 FIRST_ORDER = 1
uint8 SECOND_ORDER = 2
uint8 THIRD_ORDER = 3

uint8 label					#Not have to be used by the user
uint8 derive_order			#Can be FIRST_ORDER, SECOND_ORDER , THIRD_ORDER
uint8 comparison			#Can be EQUAL,INF,SUP
float64 value