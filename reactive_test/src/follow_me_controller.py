from tulip import spec, synth , dumpsmach

import cPickle as pickle
import sys

sys.setrecursionlimit(1500000)

dim = 100
number_cell = dim*dim -1

#sys_target_point = [58,46,76,72,51,54,43,13,31]
#env_target_point = [20,2,70,92,97,79,29,7]
enabled_dir_4 = [(-1,0),(1,0),(0,1),(0,-1)]
enabled_dir_4_diag = [(-1,1),(1,1),(-1,-1),(1,-1)]
#fixed_obstacle = [73, 56, 42, 75, 27, 34, 88] #10
#fixed_obstacle = [1765, 1280, 1112, 1775, 535, 872, 2142] #50
#fixed_obstacle = [1132, 824, 730, 1140, 348, 578, 1394] #40
#fixed_obstacle = [286, 212, 185, 290, 94, 149, 357] #20
#fixed_obstacle = [639, 468, 397, 645, 201, 313, 775] #30
#max_obstacle_size = 2

margin = 6
env_size = 3
sys_size = 3
#allow_move = 4
# margin = 2
# env_size = 0
# sys_size = 0
# #allow_move = 4


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
				if (row >= 0 and row <dim and column >=0 and column < dim and (curr_row!=row or curr_column!=column) and ((abs(curr_column-column) == distance) or (abs(curr_row - row) == distance))):
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
# for iter1 in sys_target_point:
# 	positions = get_point_strict_dist(iter1,1,strict_dist = False)
# 	for state in positions:
# 		env_prog |= {'(env1 = '+str(state)+ ')'}


#print get_point_strict_dist(0,2)
#System definitions variables and specifications
sys_vars = {}
sys_vars['loc'] = (0, number_cell)
#sys_vars['obs_close'] = (0, 1)
sys_init = set()
sys_safe = set()
sys_prog = set()
# for state in sys_target_point :
# 	sys_prog |= {'loc = '+str(state)}

#Handle fixed obstcale constraints
sys_close_obs_pos = []
map_state_obs = dict()
# for origin in fixed_obstacle:
# 	#origin = x*dim + y;
# 	denied_state = get_point_strict_dist(origin,max_obstacle_size,strict_dist=False)
# 	denied_state.append(origin)
# 	if len(denied_state) != 0:
# 		sys_safe |= {'! ('+get_spec_list_neighbor('loc',denied_state)+')'}
# 		env_safe |= {'! ('+get_spec_list_neighbor('env1',denied_state)+')'} 
# 	voisi = get_point_strict_dist(origin,max_obstacle_size+1)
# 	sys_close_obs_pos.extend(voisi)
# 	for elem in voisi:
# 		if not map_state_obs.has_key(elem):
# 			map_state_obs[elem] = list()
# 		map_state_obs[elem].extend(voisi)

#The  sys and the env should move to adjacent cells (cells at 1 dist N,S,W,E from them)
valid_env_list = list()
for iter1 in range(dim*dim):
	#sys specifications
	#next_state_sys = getPointFromDist(iter1,gen_enabled_dir(enabled_dir,1,strict_dist=False))
	next_state_sys = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4_diag,1,strict_dist=False))
	next_state_sys = list(set(next_state_sys).difference(set(map_state_obs.get(iter1,[]))))
	if(map_state_obs.has_key(iter1)):
		next_state_sys.extend(getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4,max_obstacle_size+1,strict_dist=False)))
	else:
		next_state_sys.extend(getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4,1,strict_dist=False)))
	sys_safe |= {'(loc ='+str(iter1) +') -> X ((loc = '+str(iter1)+') || '+get_spec_list_neighbor('loc',next_state_sys)+')'}

	#Environment specs
	neighbor_env = get_point_strict_dist(iter1,env_size+sys_size+margin)
	if len(neighbor_env) != 8*(env_size+sys_size+margin):
		env_safe |= {'! (env1 ='+str(iter1)+')'}
	else:
		valid_env_list.append(iter1)
		#next_state_env = getPointFromDist(iter1,gen_enabled_dir(enabled_dir ,1,strict_dist=False))
		next_state_env = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4_diag ,1,strict_dist=False))
		next_state_env = list(set(next_state_env).difference(set(map_state_obs.get(iter1,[]))))
		next_state_env.extend(getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4,1,strict_dist=False)))
		next_valid_env = list()
		for elem in next_state_env:
			neighbor_env = get_point_strict_dist(elem,(env_size+sys_size+margin))
			if len(neighbor_env) == 8*(env_size+sys_size+margin):
				next_valid_env.append(elem)
		env_safe |= {'(env1 ='+str(iter1) +') -> X ((env1 = '+str(iter1)+') || '+get_spec_list_neighbor('env1',next_valid_env)+')'}
	#sys_safe |= {'(loc ='+str(iter1) +') -> X ((loc = '+str(iter1)+') || '+get_spec_list_neighbor('loc',next_state_sys)+')'}



# sys_close_obs_pos = list(set(sys_close_obs_pos))
# sys_safe |= {get_spec_list_neighbor('loc',sys_close_obs_pos)+' -> (obs_close = 1)'}
# sys_safe |= {'!('+get_spec_list_neighbor('loc',sys_close_obs_pos)+') -> (obs_close = 0)'}
#The sys have always to be one cell away from the env (exchanges between quad not handle here)
for iter1 in valid_env_list:
	#next_state_all = get_point_strict_dist(iter1,margin+env_size+sys_size)
	#sys_safe |= {'(env1 = '+str(iter1) +') -> ('+get_spec_list_neighbor('loc',next_state_all)+')'}
	# next_state_all.extend(get_point_strict_dist(iter1,margin+env_size+sys_size+1))
	# next_state_all.extend(get_point_strict_dist(iter1,margin+env_size+sys_size-1))
	# next_state_all.extend(get_point_strict_dist(iter1,margin+env_size+sys_size+2))
	# next_state_all.extend(get_point_strict_dist(iter1,margin+env_size+sys_size-2))
	# next_state_all.extend(get_point_strict_dist(iter1,margin+env_size+sys_size+3))
	# next_state_all.extend(get_point_strict_dist(iter1,margin+env_size+sys_size-3))
	next_state = getPointFromDist(iter1,gen_enabled_dir(enabled_dir,margin+env_size+sys_size,strict_dist=True))
	sys_safe |= {'(env1 = '+str(iter1) +') -> ('+get_spec_list_neighbor('loc',next_state)+')'}
	# sys_safe |= {'(obs_close = 1) && (env1 = '+str(iter1) +') -> ('+get_spec_list_neighbor('loc',next_state_all)+')'}
	# sys_safe |= {'(obs_close = 0) && (env1 = '+str(iter1) +') -> ('+get_spec_list_neighbor('loc',next_state)+')'}


#Don't allow sys and env to exchange position


specs = spec.GRSpec(env_vars,sys_vars,env_init,sys_init,env_safe,sys_safe,env_prog,sys_prog) #GR(1) specification created
print specs.pretty()
#
# Controller synthesis
#
# The controller decides based on current variable values only,
# without knowing yet the next values that environment variables take.
# A controller with this information flow is known as Moore.
specs.moore = False
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
dumpsmach.write_python_case("followMeController_50_4_2_2_obs.py", ctrl, classname="FollowMeCtrl_50_obs")

# from tulip import spec, synth , dumpsmach

# import cPickle as pickle
# import sys

# sys.setrecursionlimit(150000000)

# # dim = 50
# # number_cell = dim*dim -1

# # sys_size = 2
# # env_size = 2
# # margin = 5

# dim = 30
# number_cell = dim*dim -1

# sys_size = 0
# env_size = 0
# margin = 6

# neighbor_sys_loc = 1

# enabled_dir = [(-1,0),(1,0),(0,1),(0,-1),(-1,1),(1,1),(-1,-1),(1,-1)]
# enabled_dir_4 = [(-1,0),(1,0),(0,1),(0,-1)]
# enabled_dir_4_diag = [(-1,1),(1,1),(-1,-1),(1,-1)]

# def getPointFromDist(curr_state,enabled_dir):
#     res = list()
#     row = curr_state/dim
#     column = curr_state % dim
#     for value in enabled_dir:
#         if((row + value[0] >=0) and (column+value[1]>=0) and (row+value[0]<dim) and (column+value[1] <dim)):
#             res.append((row+value[0])*dim + (column+value[1]))
#     return res

# def gen_enabled_dir(cap_dir, dist,strict_dist = True):
#     res = list()
#     for value in cap_dir:
#         if strict_dist :
#             res.append((value[0]*dist,value[1]*dist))
#         else:
#             for i in range(dist):
#                 res.append((value[0]*(i+1),value[1]*(i+1)))
#     return res

# def get_point_strict_dist(curr_state,distance,strict_dist = True):
#     curr_row = curr_state/dim
#     curr_column = curr_state % dim
#     res = list()
#     for row in range(curr_row-distance,curr_row+distance+1,1):
#         for column in range(curr_column-distance,curr_column+distance+1,1):
#             if strict_dist:
#                 if (row >= 0 and row <dim and column >=0 and column < dim and ((abs(curr_column-column) == distance) or (abs(curr_row - row) == distance))):
#                     res.append(row*dim + column)
#             else:
#                 if row >= 0 and row <dim and column >=0 and column < dim  and ((row != curr_row) or (column != curr_column)):
#                     res.append(row*dim + column)
#     return res

# def get_spec_list_neighbor(var_name, list_neighbor):
#     return ' || '.join( "("+var_name+" = "+str(value)+")" for value in list_neighbor)

# #Environment definitions variables and specifications
# env_vars = {'env1' : (0,number_cell), 'sys_delta_col' : (0,2*neighbor_sys_loc), 'sys_delta_row': (0,2*neighbor_sys_loc)}
# env_init = {'sys_delta_row = '+str(neighbor_sys_loc),'sys_delta_col = '+str(neighbor_sys_loc)}
# env_safe = set()
# env_prog = set()

# #print get_point_strict_dist(0,2)
# #System definitions variables and specifications
# sys_vars = {}
# sys_vars['loc'] = (0, number_cell)
# sys_init = set()
# sys_safe = set()
# sys_prog = set()

# #The  sys and the env should move to adjacent cells (cells at 1 dist N,S,W,E from them)
# valid_env_list = list()
# for iter1 in range(dim*dim):
# 	neighbor_env = get_point_strict_dist(iter1,(env_size+sys_size+margin))
# 	if len(neighbor_env) != 8*(env_size+sys_size+margin):
# 		env_safe |= {'! (env1 ='+str(iter1)+')'}
# 	else:
# 		valid_env_list.append(iter1)
# 		next_state_env = getPointFromDist(iter1,gen_enabled_dir(enabled_dir,1))
# 		next_valid_env = list()
# 		for elem in next_state_env:
# 			neighbor_env = get_point_strict_dist(elem,(env_size+sys_size+margin))
# 			if len(neighbor_env) == 8*(env_size+sys_size+margin):
# 				next_valid_env.append(elem)
# 		env_safe |= {'(env1 ='+str(iter1) +') -> X ((env1 = '+str(iter1)+') || '+get_spec_list_neighbor('env1',next_valid_env)+')'}
# 	#next_state_sys_diag = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4_diag,2,strict_dist=False))
# 	#next_state_sys = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4,(env_size+sys_size+margin),strict_dist = False)) #env_size+sys_size+margin+1
# 	#sys_safe |= {'(loc = '+str(iter1)+') -> X ((loc = '+str(iter1)+') || '+get_spec_list_neighbor('loc',next_state_sys)+ ' || '+get_spec_list_neighbor('loc',next_state_sys_diag)+ ')'}
# 	for row in range(2*neighbor_sys_loc+1):
# 		for col in range(2*neighbor_sys_loc+1):
# 			sys_row = iter1/dim
# 			sys_col = iter1 % dim
# 			new_sys_row = sys_row - neighbor_sys_loc + row
# 			new_sys_col = sys_col - neighbor_sys_loc + col
# 			new_state = new_sys_row*dim + new_sys_col
# 			if new_sys_col>= 0 and new_sys_col <dim and new_sys_row>=0 and new_sys_row<dim:
# 				next_state_sys_diag = getPointFromDist(new_state,gen_enabled_dir(enabled_dir_4_diag,neighbor_sys_loc+1,strict_dist=False))
# 				next_state_sys = getPointFromDist(new_state,gen_enabled_dir(enabled_dir_4,neighbor_sys_loc+1,strict_dist = False)) #env_size+sys_size+margin+1
# 				sys_safe |= {'((loc = '+str(iter1)+') && (sys_delta_row = '+str(row)+') && (sys_delta_col = '+str(col)+')) -> X ((loc = '+str(new_state)+') || '+get_spec_list_neighbor('loc',next_state_sys)+ ' || '+get_spec_list_neighbor('loc',next_state_sys_diag)+ ')'}
# 			else:
# 				next_state_sys_diag = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4_diag,1,strict_dist=False))
# 				next_state_sys = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4,1,strict_dist = False)) #env_size+sys_size+margin+1
# 				sys_safe |= {'((loc = '+str(iter1)+') && (sys_delta_row = '+str(row)+') && (sys_delta_col = '+str(col)+')) -> X ((loc = '+str(iter1)+') || '+get_spec_list_neighbor('loc',next_state_sys)+ ' || '+get_spec_list_neighbor('loc',next_state_sys_diag)+ ')'}

# #The sys have always to be one cell away from the env (exchanges between quad not handle here)
# for iter1 in valid_env_list:
# 	next_state = get_point_strict_dist(iter1,sys_size+env_size+margin,strict_dist=True)
# 	sys_safe |= {'(env1 = '+str(iter1) +') -> ('+get_spec_list_neighbor('loc',next_state)+')'}

# #Don't allow sys and env to exchange position


# specs = spec.GRSpec(env_vars,sys_vars,env_init,sys_init,env_safe,sys_safe,env_prog,sys_prog) #GR(1) specification created
# print specs.pretty()
# #
# # Controller synthesis
# #
# # The controller decides based on current variable values only,
# # without knowing yet the next values that environment variables take.
# # A controller with this information flow is known as Moore.
# specs.moore = True
# # Ask the synthesizer to find initial values for system variables
# # that, for each initial values that environment variables can
# # take and satisfy `env_init`, the initial state satisfies
# # `env_init /\ sys_init`.
# specs.qinit = '\E \A' # i.e., "there exist sys_vars: forall sys_vars"
# #assert not specs.sys_safe

# ctrl = synth.synthesize('gr1c',specs)
# assert ctrl is not None, 'unrealizable'
# #print ctrl
# #print ctrl
# #if not ctrl.save('gr1.png'):
# #    print(ctrl)
# #print ctrl
# #print specs.pretty()
# dumpsmach.write_python_case("followMeController30_6_1.py", ctrl, classname="FollowCtrl30_6_1")