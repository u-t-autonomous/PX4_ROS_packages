#A constrained Value represents a constraint on x,y,z, yaw derivatives.
#For example the constraint equality : Vx = 2.0 or Vx <= 2.0 can be write with the following ConstraintValue Object :
### derive_order = FIRST_ORDER
### value = 2.0
### comparison = EQUAL or comparison = INF
# For yaw we currently support only first order constraint. And second order constraints 
#for the position. 

uint32 FIRST_ORDER = 1
uint32 SECOND_ORDER = 2
uint32 THIRD_ORDER = 3

bool bndl_valid
bool bndu_valid

uint8 label					#Not have to be used by the user
uint32 derive_order			#Can be FIRST_ORDER, SECOND_ORDER , THIRD_ORDER
float64 bndl
float64 bndu