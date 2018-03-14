#!/usr/bin/env python

import sys
import importlib

from tulip import spec
from tulip import synth
from tulip import dumpsmach
import cPickle as pickle

import os.path

#Allow neighbor directions cells where systems can move
enabled_dir_ = [(-1,0),(1,0),(0,1),(0,-1)]
enabled_dir_4 = [(-1,0),(1,0),(0,1),(0,-1)]
enabled_dir_4_diag = [(-1,1),(1,1),(-1,-1),(1,-1)]
enabled_dir = enabled_dir_4
#enabled_dir.extend(enabled_dir_4_diag)

#Utility function for getting all neighbor in a certain enabled direction
#This can be use to get neighbor, to check if we are close to a border etc ... 
def getPointFromDist(curr_state, row_number , col_number , enabled_dir = enabled_dir ):
	res = list()
	row = curr_state/col_number
	column = curr_state % col_number
	for value in enabled_dir:
		if((row + value[0] >=0) and (column+value[1]>=0) and (row+value[0]<row_number) and (column+value[1] <col_number)):
			res.append((row+value[0])*col_number + (column+value[1]))
	return res 

#Utility function for getting the list of direction at a distace dist from each direction in cap_dir
# The stric_dist =True gives back the direction at exactly dist
# stric_dist = False gives back the direction at a distance less or equal than dist
def gen_enabled_dir(cap_dir, dist , strict_dist = True):
	res = list()
	for value in cap_dir:
		if strict_dist :
			res.append((value[0]*dist,value[1]*dist))
		else:
			for i in range(dist):
				res.append((value[0]*(i+1),value[1]*(i+1)))
	return res

#Utility function that return the list of state at distance distance from the curr_state
# strict_dist = False in case we want state at distance less than distance
# strict_dist = True in case we want state at distance strict equal to distance
def get_point_strict_dist(curr_state , distance , row_number , col_number , strict_dist = True):
	curr_row = curr_state/col_number
	curr_column = curr_state % col_number
	res = list()
	for row in range(curr_row-distance,curr_row+distance+1,1):
		for column in range(curr_column-distance,curr_column+distance+1,1):
			if strict_dist:
				if (row >= 0 and row <row_number and column >=0 and column < col_number and (curr_row!=row or curr_column!=column) and ((abs(curr_column-column) == distance) or (abs(curr_row - row) == distance))):
					res.append(row*col_number + column)
			else:
				if row >= 0 and row <row_number and column >=0 and column < col_number  and ((row != curr_row) or (column != curr_column)):
					res.append(row*col_number + column)
	return res

#Get in LTL a specification for the variable with var name as the name and those specifications concern the states stored in list_neighbor
def get_spec_list_neighbor(var_name, list_neighbor , ops=' || '):
	return ops.join( "("+var_name+" = "+str(value)+")" for value in list_neighbor)


def handle_static_obstacles(list_obstacles ,  sys_name ):
	sys_obstacle =  set()
	env_obstacle = set()
	for obstacles in list_obstacles:
		sys_obstacle |= {'! ({sys_name} = {obstacle_state})'.format(sys_name=sys_name , obstacle_state=obstacles)}
		env_obstacle |= {'! ({env_name} = {obstacle_state})'.format(env_name='env0' , obstacle_state=obstacles)}
	return sys_obstacle , env_obstacle

def allow_moves(row_number , col_number , sys_name ,list_obstacles):
	sys_moves = set()
	env_moves = set()
	possibilities = (set([i for i in range(col_number*row_number)]) - set(list_obstacles))
	for elem in possibilities:
		neighbor_4 = getPointFromDist(elem , row_number , col_number , enabled_dir_)
		if len(neighbor_4) != 4:
			valid_next = list()
			valid_next.extend(getPointFromDist(elem , row_number , col_number))
			neighbor_4_next = getPointFromDist(elem , row_number , col_number , gen_enabled_dir(enabled_dir_ , 2 , strict_dist=True))
			for val in neighbor_4_next:
				if len(getPointFromDist(elem , row_number , col_number , enabled_dir_)) != 4:
					valid_next.append(val)
			valid_next = list(set(valid_next))
			sys_moves |= {'({sys_name}={state}) -> X (({sys_name} = {state}) || ({list_valid_neighbor}))'.format(sys_name=sys_name , state=elem , list_valid_neighbor= get_spec_list_neighbor(sys_name,valid_next))}
			env_moves |= {'({env_name}={state}) -> X (({env_name} = {state}) || ({list_valid_neighbor}))'.format(env_name='env0' , state=elem , list_valid_neighbor= get_spec_list_neighbor('env0',getPointFromDist(elem , row_number , col_number , enabled_dir)))}
			continue
		neighbor = getPointFromDist(elem , row_number , col_number)
		#neighbor_diag = getPointFromDist(elem,row_number,col_number,enabled_dir_4_diag)
		valid_state = neighbor
		# for neighbor_elem in neighbor:
		# 	if len (set(getPointFromDist(neighbor_elem , row_number , col_number )) & set(list_obstacles)) == 0:
		# 		valid_state.append(neighbor_elem)
		# 	elif neighbor_elem not in neighbor_diag:
		# 		valid_state.append(neighbor_elem)
		sys_moves |= {'({sys_name}={state}) -> X (({sys_name} = {state}) || ({list_valid_neighbor}))'.format(sys_name=sys_name , state=elem , list_valid_neighbor= get_spec_list_neighbor(sys_name,valid_state))}
		#sys_moves |= {'({sys_name}={state}) -> X (({sys_name} = {state}) || ({list_valid_neighbor}))'.format(sys_name=sys_name , state=elem , list_valid_neighbor= get_spec_list_neighbor(sys_name,get_point_strict_dist(elem,2,row_number,col_number,strict_dist=False)))}
		env_moves |= {'({env_name}={state}) -> X (({env_name} = {state}) || ({list_valid_neighbor}))'.format(env_name='env0' , state=elem , list_valid_neighbor= get_spec_list_neighbor('env0',valid_state))}
	
	for elem in range(1,row_number-1):
		env_moves |= {'({env_name} = {state}) -> X (({env_name} = {state}) || ({env_name} = {state_plus}) || ({env_name} = {state_moins}))'.format(env_name='env1' , state=elem , state_moins=elem-1 , state_plus=elem+1)}
	env_moves |= {'({env_name} = 0) -> X( ({env_name} = 0) || ({env_name} = 1) )'.format(env_name='env1')}
	env_moves |= {'({env_name} = {last_elem}) -> X( ({env_name} = {last_elem}) || ({env_name} = {last_elem_moins}) )'.format(env_name='env1',last_elem_moins=row_number-2,last_elem=row_number-1)}
	return sys_moves , env_moves

def no_collision_env(sys_name, row_number , col_number):
	col_spec = set()
	for elem in range(row_number * col_number):
		#col_spec |= {'! ( X ( (env = {value}) && ({sys_name} = {value}) ))'.format(value=elem , sys_name=sys_name)}
		#col_spec |= {'(!( (env = {value}) && ({list_non_admissible}) ))'.format(value=elem , list_non_admissible=get_spec_list_neighbor(sys_name,getPointFromDist(elem , row_number , col_number)))}
		col_spec |= {'( (env0 = {value}) -> ! (({sys_name} = {value}) || ({list_non_admissible})) )'.format(value=elem , sys_name=sys_name,list_non_admissible=get_spec_list_neighbor(sys_name,getPointFromDist(elem , row_number , col_number)))}
		#col_spec |= {'( (env1 = {value}) -> ! (({sys_name} = {value}) || ({list_non_admissible})) )'.format(value=elem , sys_name=sys_name,list_non_admissible=get_spec_list_neighbor(sys_name,getPointFromDist(elem , row_number , col_number)))}
		#col_spec |= {'! ( (env0 = {value}) && ({sys_name} = {value}) ) '.format(sys_name=sys_name,value=elem)}
	for elem in range(row_number):
		col_spec |= {'( (env1 = {value}) -> !  (({sys_name} = {value_val}) )) '.format(sys_name=sys_name,value=elem,value_val=elem*(row_number+1))}
	# for elem in range(row_number):
	# 	col_spec |= {'( (env1 = {value}) -> ! (({sys_name} = {value}) || ({list_non_admissible})) )'.format(value=elem , sys_name=sys_name,list_non_admissible=get_spec_list_neighbor(sys_name,getPointFromDist(elem *(row_number+1), row_number , col_number,[(1,0),(-1,0)])))}
	
	return col_spec

def handle_target_point(row_number , col_number , list_target , sys_name , sup_target_list):
	set_target = set()
	for stage in range(len(list_target)):
		set_target |= {'(stage_{sys_name} = {stage}) && (! ({sys_name} = {position})) && (follow_complete = 1) -> X ((stage_{sys_name} = {stage}) )'.format(sys_name = sys_name , stage = stage, position = list_target[stage],target=len(sup_target_list))}
		set_target |= {'(stage_{sys_name} = {stage}) && ({sys_name} = {position}) && (follow_complete = 1) -> X ((stage_{sys_name} = {stage_1}) )'.format(sys_name = sys_name , stage = stage, position = list_target[stage], stage_1 = (stage+1),target=len(sup_target_list))}
		#set_target |= {'(stage_{sys_name} = {stage}) && follow_target = '}
		for target in range(len(sup_target_list)):
			set_target |= {'(follow_complete = 0) && (stage_{sys_name} = {stage}) && (! ({sys_name} = {position})) && (target_{sys_name} = {target}) -> X ((follow_complete = 0) && (stage_{sys_name} = {stage})) '.format(sys_name=sys_name,stage = stage,last_target=len(sup_target_list),target=target,position=sup_target_list[target])}
			set_target |= {'(follow_complete = 0) && (stage_{sys_name} = {stage}) &&  ({sys_name} = {position}) && (target_{sys_name} = {target})  -> X (((!(target_{sys_name}={last_target})) -> (follow_complete = 1)) && (stage_{sys_name} = {stage}))'.format(sys_name=sys_name,stage = stage,target=target,last_target=len(sup_target_list),position=sup_target_list[target])}

	for target in range(len(sup_target_list)):
	 	set_target |= {'(stage_{sys_name} = {stage}) && (! ({sys_name} = {position})) && (target_{sys_name} = {target}) -> X ((follow_complete = 0))'.format(sys_name=sys_name,stage = len(list_target),target=target,position=sup_target_list[target])}
	 	set_target |= {'(stage_{sys_name} = {stage}) &&  ({sys_name} = {position}) && (target_{sys_name} = {target}) -> X (((!(target_{sys_name}={last_target})) -> (follow_complete = 1)))'.format(sys_name=sys_name,stage = len(list_target),target=target,last_target=len(sup_target_list),position=sup_target_list[target])}
	#set_target |= {'(stage_{sys_name} = {stage})  && (target_{sys_name} = {target}) -> X ((stage_{sys_name} = {stage}) && (follow_target = 0))'.format(sys_name = sys_name , stage = len(list_target),target=len(sup_target_list))}
	return set_target

def solve_problem(init_state_sys , init_state_env, file_name, row_number , col_number , list_target , sys_name , sup_target_list , list_obstacles):
	print row_number
	print col_number
	print sys_name
	print sup_target_list
	print list_target
	#Environment variable
	env_vars = {}
	env_vars['env0'] = (0, row_number * col_number -1)
	env_vars['env1'] = (0, row_number-1)
	env_vars['target_'+sys_name] = (0, len(sup_target_list))
	
	#Envrionment init value
	#env_init = {'target_{sys_name} = {last_value}'.format(sys_name=sys_name, last_value=len(sup_target_list))}
	env_init = set()
	env_init |= {'target_{sys_name} = {last_value}'.format(sys_name=sys_name, last_value=len(sup_target_list))}
	env_init |= {'env0 = '+str(init_state_env)}
	env_init |= {'env1 = '+str(0)}
	#env_init = {'!(target_{sys_name} = {last_value})'.format(sys_name=sys_name, last_value=0)}
	#env_init = {'!(target_{sys_name} = {last_value})'.format(sys_name=sys_name, last_value=1)}

	#Environment prog
	env_prog = set()
	#env_prog ={'target_{sys_name} = {last_value}'.format(sys_name=sys_name, last_value=0)}
	#env_prog ={'target_{sys_name} = {last_value}'.format(sys_name=sys_name, last_value=1)}
	#env_prog ={"(target_{sys_name} = {last_value})".format(sys_name=sys_name , last_value=0)}
	#env_prog = {"! (target_{sys_name} = {last_value})".format(sys_name=sys_name , last_value=len(sup_target_list))}
	env_prog |= {' env0 = '+str(init_state_env)}
	env_prog |= {' env1 = 0'}
	
	env_safe = set()
	env_safe |= {'(target_{sys_name} ={last_value}) -> X (!(target_{sys_name} = {last_value}))'.format(sys_name=sys_name, last_value=len(sup_target_list))}
	for elem in range(len(sup_target_list)):
		env_safe |= {'(target_{sys_name} ={elem}) -> X ((target_{sys_name} = {elem}) || (target_{sys_name} = {last_value}))'.format(sys_name=sys_name,elem=elem,last_value=len(sup_target_list))}
	
	# for elem in range(row_number ):
	# 	env_safe |= {'! ( (env1 = {value}) && (env0 = {value_val}) )'.format(value=elem , value_val= elem*(row_number+1))}
	# for elem in range(row_number ):
	# 	env_safe |= {'! ( (env1 = {value}) && (env0 = {value_val}) )'.format(value=elem , value_val= elem*(row_number+1))}
	#system variable
	sys_vars = {}
	sys_vars[sys_name] = (0 , (row_number * col_number) -1)
	#sys_vars['follow_target'] = (0,1)
	sys_vars['follow_complete'] = (0,1)
	sys_vars['stage_'+sys_name] = (0, len(list_target))

	#system init variable
	sys_init = set()
	sys_init |= {"(stage_{sys_name} = {init_value})".format(sys_name=sys_name , init_value=0)}
	#sys_init |= {'follow_target = 0'}
	sys_init |= {'follow_complete = 1'}
	sys_init |= {sys_name + ' = '+ str(init_state_sys)}

	#System prog
	sys_prog = set()
	sys_prog |= {"(stage_{sys_name} = {last_value})".format(sys_name=sys_name,last_value=len(list_target))}

	#System safety
	sys_safe = set()
	for elem in range(len(list_target)):
		sys_safe |= {'(stage_{sys_name} = {value}) ->  X ((stage_{sys_name} = {value}) || (stage_{sys_name} = {value_1}))'.format(sys_name=sys_name,value=elem, value_1=elem+1)}
	sys_safe |= {'(stage_{sys_name} = {last_value}) ->  X ( (stage_{sys_name} = 0) )'.format(sys_name=sys_name, last_value=len(list_target))}
	sys_safe |= {' (target_{sys_name} = {last_value}) ->  X (follow_complete = 0)'.format(sys_name=sys_name,last_value=len(sup_target_list))}
	#sys_safe |= {' ! (target_{sys_name} = {last_value}) && X (follow_complete = 1) ->  X (follow_complete = 0)'.format(sys_name=sys_name,last_value=len(sup_target_list))}
	#sys_safe |= {'! (target_{sys_name} = {last_value}) -> (follow_target = 1)'.format(sys_name=sys_name,last_value=len(sup_target_list))}
	#sys_safe |= {'(!(target_{sys_name} = {last_value})) -> (follow_target = 1)'.format(sys_name=sys_name,last_value=len(sup_target_list))} #To modify

	(moves_sys , moves_env) = allow_moves(row_number , col_number , sys_name ,list_obstacles)
	(obs_sys , obs_env) = handle_static_obstacles(list_obstacles , sys_name)

	sys_safe |= moves_sys
	sys_safe |= obs_sys
	sys_safe |= no_collision_env(sys_name, row_number , col_number)
	sys_safe |= handle_target_point(row_number , col_number , list_target , sys_name , sup_target_list)

	env_safe |= moves_env
	env_safe |= obs_env

	specs = spec.GRSpec(env_vars ,sys_vars, env_init ,sys_init , env_safe, sys_safe ,env_prog ,sys_prog)
	print specs.pretty()
	specs.moore = True
	specs.qinit = '\E \A'
	ctrl = synth.synthesize('gr1c' , specs)
	assert ctrl is not None, 'Unrealizable'
	#print ctrl
	dumpsmach.write_python_case(file_name+'_controller.py', ctrl, classname=file_name+'Controller')

class AgentController:
	def __init__(self , init_state , init_state_env , file_name, row_number , col_number , list_target , sys_name , sup_target_list , list_obstacles ):
		self.file_name = file_name
		self.row_number = row_number
		self.col_number = col_number
		self.list_target = list_target
		self.sys_name = sys_name
		self.sup_target_list = sup_target_list
		self.list_obstacles = list_obstacles
		self.init_state = init_state
		self.init_state_env = init_state_env

	def createController(self):
		solve_problem(self.init_state, self.init_state_env,self.file_name, self.row_number , self.col_number , self.list_target , self.sys_name , self.sup_target_list , self.list_obstacles)

	def getController(self):
		if not os.path.isfile(self.file_name+'_controller.py'):
			self.createController()
		controller_module = importlib.import_module(self.file_name+'_controller')
		tulip_control_class_ = getattr(controller_module , self.file_name+'Controller')
		tulip_control = tulip_control_class_()
		return tulip_control

if __name__ == "__main__":
	file_name = 'test'

	row_number = 5
	col_number = 5

	list_target = [7]
	sys_name = 'Quad9'
	sup_target_list = [5,17 , 4]
	list_obstacles =[8,16]

	agent = AgentController(4,20,file_name , row_number , col_number , list_target , sys_name , sup_target_list , list_obstacles)
	agent.createController()
	controller = agent.getController()

	for elem in (range(50)):
		target = int(raw_input('target ind : '))
		env0 = int(raw_input('env0 environment '))
		env1 = int(raw_input('env1 environment '))
		print controller.move(env1,env0,target)
