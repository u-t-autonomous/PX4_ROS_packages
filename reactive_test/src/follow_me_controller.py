from tulip import spec, synth , dumpsmach

import cPickle as pickle

dim = 10
number_cell = dim*dim -1

sys_target_point = [22,27,57,72]
#env_target_point = [20,2,70,92,97,79,29,7]
follow_distance = 2

enabled_dir = [(-1,0),(1,0),(0,1),(0,-1),(-1,1),(1,1),(-1,-1),(1,-1)]

def getPointFromDist(curr_state,enabled_dir):
	res = list()
	row = curr_state/dim
	column = curr_state % dim
	for value in enabled_dir:
		if((row + value[0] >=0) and (column+value[1]>=0) and (row+value[0]<dim) and (column+value[1] <dim)):
			res.append((row+value[0])*dim + (column+value[1]))
	return res

def gen_enabled_dir(cap_dir, dist,strict_dist = True):
	res = list()
	for value in cap_dir:
		if strict_dist :
			res.append((value[0]*dist,value[1]*dist))
		else:
			for i in range(dist):
				res.append((value[0]*(i+1),value[1]*(i+1)))
	return res

def get_point_strict_dist(curr_state,distance,strict_dist = True):
	curr_row = curr_state/dim
	curr_column = curr_state % dim
	res = list()
	for row in range(curr_row-distance,curr_row+distance+1,1):
		for column in range(curr_column-distance,curr_column+distance+1,1):
			if strict_dist:
				if (row >= 0 and row <dim and column >=0 and column < dim and ((abs(curr_column-column) == distance) or (abs(curr_row - row) == distance))):
					res.append(row*dim + column)
			else:
				if row >= 0 and row <dim and column >=0 and column < dim  and ((row != curr_row) or (column != curr_column)):
					res.append(row*dim + column)
	return res

def get_spec_list_neighbor(var_name, list_neighbor):
	return ' || '.join( "("+var_name+" = "+str(value)+")" for value in list_neighbor)

#Environment definitions variables and specifications
env_vars = {'env1' : (0,number_cell)}
env_init = set()
env_safe = set()
env_prog = set()
# for iter1 in range(dim*dim):
# 	env_prog |= {'(env1 = '+str(iter1)+')'}
for iter1 in sys_target_point:
	positions = get_point_strict_dist(iter1,follow_distance,strict_dist = False)
	for state in positions:
		env_prog |= {'(env1 = '+str(state)+ ')'}


#print get_point_strict_dist(0,2)
#System definitions variables and specifications
sys_vars = {}
sys_vars['loc'] = (0, number_cell)
sys_init = set()
sys_safe = set()
sys_prog = set()
for state in sys_target_point :
	sys_prog |= {'loc = '+str(state)}

#The  sys and the env should move to adjacent cells (cells at 1 dist N,S,W,E from them)
for iter1 in range(dim*dim):
	next_state_env = getPointFromDist(iter1,gen_enabled_dir(enabled_dir,1))
	next_state_sys = getPointFromDist(iter1,gen_enabled_dir(enabled_dir,2,strict_dist=False))
	#next_state_sys_diag = getPointFromDist(iter1,gen([(-1,1),(1,1),(-1,-1),(1,-1)],)
	env_safe |= {'(env1 ='+str(iter1) +') -> X ('+get_spec_list_neighbor('env1',next_state_env)+')' }
	sys_safe |= {'(loc ='+str(iter1) +') -> X ((loc = '+str(iter1)+') || '+get_spec_list_neighbor('loc',next_state_sys)+')'}

#The sys have always to be one cell away from the env (exchanges between quad not handle here)
for iter1 in range(dim*dim):
	next_state = get_point_strict_dist(iter1,follow_distance)
	sys_safe |= {'(env1 = '+str(iter1) +') -> ('+get_spec_list_neighbor('loc',next_state)+')'}

#Don't allow sys and env to exchange position


specs = spec.GRSpec(env_vars,sys_vars,env_init,sys_init,env_safe,sys_safe,env_prog,sys_prog) #GR(1) specification created
#print specs.pretty()
#
# Controller synthesis
#
# The controller decides based on current variable values only,
# without knowing yet the next values that environment variables take.
# A controller with this information flow is known as Moore.
specs.moore = True
# Ask the synthesizer to find initial values for system variables
# that, for each initial values that environment variables can
# take and satisfy `env_init`, the initial state satisfies
# `env_init /\ sys_init`.
specs.qinit = '\E \A' # i.e., "there exist sys_vars: forall sys_vars"

ctrl = synth.synthesize('gr1c',specs)
assert ctrl is not None, 'unrealizable'

#if not ctrl.save('gr1.png'):
#	print(ctrl)
#print ctrl
#print specs.pretty()
dumpsmach.write_python_case("followMeController.py", ctrl, classname="FollowMeCtrl")
