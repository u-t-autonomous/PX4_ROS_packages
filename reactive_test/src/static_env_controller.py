#!/usr/bin/env python

import sys
import importlib

from tulip import spec
from tulip import synth
from tulip import dumpsmach
import cPickle as pickle


#Increase Recursion limit for high number of state problem
sys.setrecursionlimit(1500000)

#Allow neighbor directions cells where systems can move
enabled_dir_4 = [(-1,0),(1,0),(0,1),(0,-1)]
enabled_dir_4_diag = [(-1,1),(1,1),(-1,-1),(1,-1)]
enabled_dir = enabled_dir_4
enabled_dir.extend(enabled_dir_4_diag)

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

def add_margin_to_states(list_states , margin , row_number , col_number):
	res = set()
	for state in list_states:
		res.add(state)
		res |= set(get_point_strict_dist(state, margin , row_number ,col_number , strict_dist = False))
	return list(res)

def set_value_after_steps( n_steps , value):
	res = []
	for step in range(n_steps):
		res_temp = ''
		for i in range(step+1):
			res_temp += 'X ( '
		res_temp += value
		for i in range(step+1):
			res_temp += ')'
		res.append(res_temp)
	return ' || '.join(res)


#Get in LTL a specification for the variable with var name as the name and those specifications concern the states stored in list_neighbor
def get_spec_list_neighbor(var_name, list_neighbor , ops=' || '):
	return ops.join( "("+var_name+" = "+str(value)+")" for value in list_neighbor)


class AgentController:
	""" This class allows the creation of a controller with no environment variable. Attributes are:
	- row_number_				: Number of row in the grid
	- col_number_ 				: Number of column in the grid
	- sys_specs_				: dictionary where the keys are the name of the differents system and the values are an array|list of ordered target point for each system 
												and list of local neighbor 
	- static_obstacles_			: list of state where there is a static obstacle
	- sys_init_state_			: dictionnary with the initial state of some variables
	"""
	def __init__(self , controller_name, row_number_ , col_number_ , sys_specs_ , static_obstacles_  , margin_obstacle=1 , margin_system=2):
		self.controller_name = controller_name
		self.row_number = row_number_
		self.col_number = col_number_
		self.margin_system = margin_system
		self.margin_obstacle = margin_obstacle
		self.number_cell = row_number_*col_number_ - 1
		self.sys_specs = sys_specs_
		self.static_obstacles = add_margin_to_states(static_obstacles_ , margin_obstacle , row_number_ , col_number_ )
		self.sys_vars = {}
		self.sys_safe = set()
		self.sys_prog = set()
		self.sys_init = set()

	def handle_init_system(self):
		#Initialise system variables
		for sys_name, sys_values in self.sys_specs.items():
			self.sys_vars[sys_name] = (0,self.number_cell)
			self.sys_init |= {'{var_name} = {var_init_value}'.format(var_name=sys_name, var_init_value=sys_values.get('waypoints',[])[0])}

		#set init position for each system variable that have inital position specification
		#for sys_name,sys_init in self.sys_init_state.items():
		#	self.sys_init |= {'{var_name} = {var_init_value}'.format(var_name=sys_name, var_init_value=sys_init)}

	def handle_static_obstacles(self):
		for obstacles in self.static_obstacles:
			for sys_name in self.sys_specs:
				self.sys_safe |= {'! ({sys_name} = {obstacle_state})'.format(sys_name=sys_name , obstacle_state=obstacles)}


	def handle_allowed_moves(self):
		for iter1 in range(self.number_cell + 1):
			if iter1 in self.static_obstacles:
				continue
			#print iter1
			for sys_name, sys_values in self.sys_specs.items():
				surface_local_state = sys_values.get('surface',[])
				surface_state = getPointFromDist(iter1 , self.row_number , self.col_number ,  surface_local_state)
				#print sys_name , surface_state , iter1
				intersect_obstacle = list(set(surface_state) & set(self.static_obstacles))
				if len(surface_state) != len(surface_local_state) or len(intersect_obstacle) != 0:	#We remove state close to the border where the system could not stay because of his surface or close to an obstacle 
					self.sys_safe |= {'! ({sys_name} = {out_values})'.format(sys_name=sys_name , out_values= iter1)}
				else:
					next_state =  getPointFromDist(iter1 ,  self.row_number ,self.col_number)
					next_state_diag  = getPointFromDist(iter1 ,  self.row_number ,self.col_number,enabled_dir_4_diag)
					valid_state = list()
					for elem in next_state :
						surface_state_elem = getPointFromDist(elem , self.row_number ,self.col_number , surface_local_state)
						intersect_obstacle_elem = list(set(surface_state_elem) & set(self.static_obstacles))
						if len(surface_state_elem) != len(surface_local_state) or len(intersect_obstacle_elem) !=0:
							self.sys_safe |= {'! ({sys_name} = {out_values})'.format(sys_name=sys_name , out_values= elem)}
						else:
							neighbor_1_elem = get_point_strict_dist(elem , 1,self.row_number , self.col_number)
							intersect_obstacle_elem_neigh = list(set(neighbor_1_elem) & set(self.static_obstacles))
							if len(intersect_obstacle_elem_neigh) == 0:
								valid_state.append(elem)
							elif elem not in next_state_diag:
								valid_state.append(elem)


					self.sys_safe |= {'({sys_name}={state}) -> X (({sys_name} = {state}) || ({list_valid_neighbor}))'.format(sys_name=sys_name , state=iter1 , list_valid_neighbor= get_spec_list_neighbor(sys_name,valid_state))}


	def follow_targets(self):
		sys_prog_spec = []
		for sys_name, sys_values in self.sys_specs.items():
			targets_list = sys_values.get('waypoints',[])
			self.sys_vars['stage_'+str(sys_name)] = (0,len(targets_list))
			self.sys_init |= {'stage_{sys_name} = 0'.format(sys_name=sys_name)}
			sys_prog_spec.append('(stage_{sys_name} = {value})'.format(sys_name=sys_name , value= len(targets_list)))
			#self.sys_prog |= {'stage_{sys_name} = {last_stage}'.format(sys_name=sys_name , last_stage = len(targets_list)-1 )}
			for stage in range(len(targets_list)):
				self.sys_safe |= {'(stage_{sys_name} = {stage}) && ! ({sys_name} = {position}) -> X (stage_{sys_name} = {stage})'.format(sys_name = sys_name , stage = stage, position = targets_list[stage])}
				self.sys_safe |= {'(stage_{sys_name} = {stage}) && ({sys_name} = {position}) -> X (stage_{sys_name} = {stage_1})'.format(sys_name = sys_name , stage = stage, position = targets_list[stage], stage_1 = (stage+1))}
				#if( stage != len(targets_list) -1):
				#self.sys_safe |= {'(stage_{sys_name} = {stage}) -> {next_values}'.format(sys_name= sys_name , stage=stage , next_values = set_value_after_steps(n, 'stage_'+sys_name+' = '+ str((stage+1))))}
		self.sys_prog |= {' && '.join(sys_prog_spec)}
	
	def no_collision(self):
		for iter1 in range(self.number_cell +1):
			already_solve = list()
			#print iter1
			for sys_name , sys_values in self.sys_specs.items():
				already_solve.append(sys_name)
				if len(already_solve) == len(self.sys_specs):
					continue
				surface_dir = sys_values.get('surface',[])
				surface_state = add_margin_to_states(getPointFromDist(iter1 , self.row_number , self.col_number , surface_dir), self.margin_system, self.row_number ,self.col_number)
				surface_state.extend(get_point_strict_dist(iter1 , self.margin_system , self.row_number , self.col_number))
				non_admissible_state = dict()
				no_one_found = False
				rayon = 1
				while not no_one_found:
					for sys_sub , sys_values_sub in self.sys_specs.items():
						if sys_sub in already_solve:
							continue
						if non_admissible_state.get(sys_sub) == None:
							non_admissible_state[sys_sub] = set()
							non_admissible_state[sys_sub].add(iter1)
						surface_dir_sub = sys_values_sub.get('surface',[])
						neighbor_state = get_point_strict_dist(iter1 , rayon , self.row_number , self.col_number)
						for state in neighbor_state:
							surface_state_sub =  add_margin_to_states(getPointFromDist(state , self.row_number ,self.col_number ,surface_dir_sub),self.margin_system , self.row_number ,self.col_number)
							surface_state_sub.extend(get_point_strict_dist(state,self.margin_system, self.row_number , self.col_number))
							if len(set(surface_state) & set(surface_state_sub)) != 0:
								non_admissible_state[sys_sub].add(state)
								no_one_found = False
							else:
								no_one_found = True
					rayon = rayon + 1

				# for iter2 in range(self.number_cell +1):
				# 	for sys_sub , sys_values_sub in self.sys_specs.items():
				# 		if sys_sub in already_solve :
				# 			continue
				# 		if non_admissible_state.get(sys_sub) == None:
				# 			non_admissible_state[sys_sub] = set()
				# 			non_admissible_state[sys_sub].add(iter1)
				# 		surface_dir_sub = sys_values_sub.get('surface',[])
				# 		surface_state_sub =  add_margin_to_states(getPointFromDist(iter2 , self.row_number ,self.col_number ,surface_dir_sub),self.margin_system , self.row_number ,self.col_number)
				# 		surface_state_sub.extend(get_point_strict_dist(iter2,self.margin_system, self.row_number , self.col_number))
				# 		if len(set(surface_state) & set(surface_state_sub)) != 0:
				# 			non_admissible_state[sys_sub].add(iter2)
				self.sys_safe |= {'! (({sys_name} = {state}) && ({denied_state}))'.format(sys_name=sys_name , state=iter1 , denied_state= ' || '.join( get_spec_list_neighbor(k,list(v)) for (k,v) in non_admissible_state.items()))}
	
	def create_controller(self):
		self.handle_init_system()
		self.handle_static_obstacles()
		self.handle_allowed_moves()
		self.follow_targets()
		#self.no_collision()
		specs = spec.GRSpec({} ,self.sys_vars,set() , self.sys_init , set() , self.sys_safe , set() , self.sys_prog)
		#print specs.pretty()
		specs.moore = True
		specs.qinit = '\E \A'
		ctrl = synth.synthesize('gr1c' , specs)
		assert ctrl is not None, 'Unrealizable'
		dumpsmach.write_python_case(self.controller_name+'_controller.py', ctrl, classname=self.controller_name+'Controller')

	def get_trajectory(self):
		res = dict()
		for sys_name in self.sys_specs:
			res[sys_name] = list()
		controller_module = importlib.import_module(self.controller_name+'_controller')
		tulip_control_class_ = getattr(controller_module , self.controller_name+'Controller')
		tulip_control = tulip_control_class_()
		stageComplete = False
		while not stageComplete:
			next_moves = tulip_control.move()
			stageComplete = True
			for sys_name,sys_values in self.sys_specs.items():
				if next_moves['stage_'+sys_name] != len(sys_values['waypoints']):
					stageComplete = False
			if not stageComplete:
				for sys_name in self.sys_specs:
					res[sys_name].append(next_moves[sys_name])
		return res

	def get_trajectory_after(self,number_iteration):
		res = dict()
		controller_module = importlib.import_module(self.controller_name+'_controller')
		tulip_control_class_ = getattr(controller_module , self.controller_name+'Controller')
		tulip_control = tulip_control_class_()
		for elem in range(number_iteration):
			next_moves = tulip_control.move()
			for sys_name in next_moves:
				if "stage" in sys_name:
					continue
				if res.get(sys_name) is None:
					res[sys_name] = []
				res[sys_name].append(next_moves[sys_name])
		return res

if __name__ == "__main__":

	name = "test"
	row_number =50
	col_number = 50
	sys_specs = dict()
	sys_specs['sys1'] = {'waypoints' : [4 , 8 , 12 , 16 ,19] , 'surface': []}
	sys_specs['sys2'] = {'waypoints' : [0 , 6 , 12 , 18 , 15] , 'surface': []}
	#sys_specs['sys3'] = {'waypoints' : [15 , 16 , 17 , 18 ] , 'surface': []}
	static_obstacles = [14 , 10 ]
	#static_obstacles = []
	#sys_init_state = {'sys1': 4 , 'sys2': 0}
	sys_init_state = {'sys1': 4}


	agentController = AgentController(name , row_number ,col_number , sys_specs , static_obstacles , 0 , 0)

	#agentController.handle_init_system()
	#agentController.handle_static_obstacles()

	#agentController.handle_allowed_moves()
	#agentController.follow_targets()
	#agentController.no_collision()


	# print agentController.sys_vars
	# print '\n'.join( k for k in agentController.sys_init)
	# print agentController.static_obstacles
	#print '\n'.join( k for k in agentController.sys_safe)

	agentController.create_controller()
	#traj = agentController.get_trajectory()

	#print traj
	#print ""
	#print ""

	#print reduce_state([4,9,14,19,13,7,11,15,10,5,0] , 1 , agentController.row_number , agentController.col_number)


