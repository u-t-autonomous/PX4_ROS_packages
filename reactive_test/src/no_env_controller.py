from tulip import spec, synth , dumpsmach

import cPickle as pickle
import sys

sys.setrecursionlimit(150000000)

dim = 20
number_cell = dim*dim -1

def set_stage(nb_stage, list_position , ind):
	plan = set()
	for stage in range(nb_stage):
		plan |= {('((stage = {stage}) && !(loc{ind} = {position})) -> X (stage = {stage})'.format(ind=ind,stage=stage,position=list_position[stage]))}
		plan |= {('((stage = {stage}) && (loc{ind} = {position})) -> X (stage = {stage_1})'.format(ind=ind,stage=stage,position=list_position[stage],stage_1=(stage+1)%nb_stage))}
	return plan

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

enabled_dir_4 = [(-1,0),(1,0),(0,1),(0,-1)]
fix_obstacle = [3,6,13,16]
list_pos_quad1 = [0,11,18,9]
list_pos_quad2 = [2,10,12,4]
list_pos_quad3 = [23,12,14,2]
nb_stage = 4
stage_init = 0

env_vars = set()
env_init = set()
env_safe = set()
env_prog = set()

sys_vars = {}
sys_safe = set()
sys_prog = set()
sys_init = set()

sys_vars['loc1'] = (0, number_cell)
sys_vars['loc2'] = (0, number_cell)
sys_vars['loc3'] = (0, number_cell)
sys_vars['stage'] = (0, nb_stage-1)
#sys_vars['stage2'] = (0, nb_stage-1)
#sys_vars['stage3'] = (0, nb_stage-1)

sys_init |= {'stage = '+str(stage_init)}
#sys_init |= {'stage2 = '+str(stage_init)}
#sys_init |= {'stage3 = '+str(stage_init)}

sys_prog |= {'stage = '+str(nb_stage-1)}
#sys_prog |= {'stage1 = '+str(nb_stage-1)}
#sys_prog |= {'stage1 = '+str(nb_stage-1)}

sys_safe |= set_stage(nb_stage, list_pos_quad1 , 1)
sys_safe |= set_stage(nb_stage, list_pos_quad2 , 2)
sys_safe |= set_stage(nb_stage, list_pos_quad3 , 3)

for iter1 in range(len(fix_obstacle)):
	sys_safe |= {'! ( loc1 = '+str(fix_obstacle[iter1])+' )'}
	sys_safe |= {'! ( loc2 = '+str(fix_obstacle[iter1])+' )'}
	sys_safe |= {'! ( loc3 = '+str(fix_obstacle[iter1])+' )'}

for iter1 in range(dim * dim):
	next_state = getPointFromDist(iter1,gen_enabled_dir(enabled_dir_4,1))
	sys_safe |= {'(loc1 = '+str(iter1) +') -> X ((loc1 = '+str(iter1)+') || '+get_spec_list_neighbor('loc1',next_state)+')'}
	sys_safe |= {'(loc2 = '+str(iter1) +') -> X ((loc2 = '+str(iter1)+') || '+get_spec_list_neighbor('loc2',next_state)+')'}
	sys_safe |= {'(loc3 = '+str(iter1) +') -> X ((loc3 = '+str(iter1)+') || '+get_spec_list_neighbor('loc3',next_state)+')'}
	#No collision between sys and environment
	sys_safe |= {'! ( (loc1 = '+str(iter1) +') && (loc2 = '+str(iter1)+') && (loc3 = '+str(iter1)+') )'}

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
#assert not specs.sys_safe

ctrl = synth.synthesize('gr1c',specs)
assert ctrl is not None, 'unrealizable'
print ctrl
#print ctrl
#if not ctrl.save('gr1.png'):
#    print(ctrl)
#print ctrl
#print specs.pretty()
dumpsmach.write_python_case("sys3_controller.py", ctrl, classname="NoEnvController")