from tulip import spec, synth , dumpsmach

import cPickle as pickle

def set_stage(nb_stage, list_position):
	plan = set()
	for stage in range(nb_stage):
		plan |= {('((stage = '+str(stage) +') && !(loc = '+str(list_position[stage])+')) -> X (stage = '+str(stage)+')')}
		plan |= {('((stage = '+str(stage) +') && (  loc = '+str(list_position[stage])+')) -> X (stage = '+str((stage+1)%nb_stage)+')')}
	return plan

dim = 5
number_cell = dim*dim -1
env_row = 3
#env_init_pos = 0
#sys_init_pos = 12

nb_stage = 5
list_position =[12, 18 , 16 , 17 , 15 ]
stage_init = 0
#Environment definitions variables and specifications
env_vars = {'env1' : (0,dim-1)}
env_init = set() 	#{'env1 = '+str(env_init_pos)}
env_safe = set()
env_prog = set()
env_prog |= {'env1 = '+str(dim-1)}

for iter1 in range(dim-2):
	env_safe |= {('(env1 = '+str(iter1 + 1)+') -> X ((env1 = '+str(1+iter1-1)+') || (env1 = '+str(1+iter1+1)+'))')}
env_safe |= {('(env1 = 0) -> X (env1 = 1)')}
env_safe |= {('(env1 = '+str(dim-1)+') -> X (env1 = '+str(dim-2)+')')}

#System definitions variables and specifications

sys_vars = {}
sys_vars['loc'] = (0, number_cell)
sys_vars['stage'] = (0, nb_stage-1)

#sys_init = {'loc = '+str(sys_init_pos),'stage = 0'}
sys_init = {'stage = '+str(stage_init)}
sys_safe = set()
sys_prog = set()

#The transition of stage for the sys robot
sys_safe |= set_stage(nb_stage,list_position)
#print(set_stage(nb_stage,list_position))

#Sys robot should go infinitely often to stage 7
sys_prog |= {'stage = 4'}
sys_prog |= {'stage = 0'}

## The environment robots and the controlled robots can move anywhere in the grid
## but only to some of his adjacent cells at each step
for iter1 in range(dim*dim):
	if(iter1 % dim != 0 ) and (iter1 % dim != dim-1) and (iter1/dim != 0) and (iter1/dim != dim-1) : #Case inside the grid
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1+1) + ') || (loc='+str(iter1-1)+') || (loc='+str(iter1-dim)+') || (loc = '+str(iter1+dim)+'))')}
	elif (iter1 % dim == 0) and (iter1 / dim  !=0) and (iter1/dim != dim-1): #Case on the left corner of the grid excep left up and down
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1+1) + ') || (loc='+str(iter1-dim)+') || (loc = '+str(iter1+dim)+'))')}
	elif (iter1 % dim == dim-1) and (iter1/dim !=0) and (iter1/dim != dim-1): #Case on the right corner of the grid except right up and down
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1-1) + ') || (loc='+str(iter1-dim)+') || (loc = '+str(iter1+dim)+'))')}
	elif (iter1 %dim != 0 ) and (iter1%dim  != dim-1) and (iter1 / dim == dim-1):#Top corner of the grid
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1+1) + ') || (loc='+str(iter1-1)+') || (loc='+str(iter1-dim)+'))')}
	elif (iter1%dim !=0 ) and (iter1%dim !=dim-1) and (iter1/dim == 0) :#Bottom corner of the grid
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1+1) + ') || (loc='+str(iter1-1)+') || (loc='+str(iter1+dim)+'))')}
	elif (iter1 == 0) :
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1+1) + ') || (loc='+str(iter1+dim)+'))')}
	elif (iter1 == dim-1) :
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1-1) + ') || (loc='+str(iter1+dim)+'))')}
	elif (iter1 == dim*(dim-1)):
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1+1) + ') || (loc='+str(iter1-dim)+'))')}
	elif (iter1 == dim*dim -1):
		sys_safe |= {('(loc = '+str(iter1)+') -> X((loc = '+str(iter1)+') || (loc='+str(iter1-1) + ') || (loc='+str(iter1-dim)+'))')}

#The sys should not enter in collision with environment

for iter1 in range(dim):
	sys_safe |= {('!(env1 = '+str(iter1)+') || !(loc = '+str(iter1+ env_row*dim)+')')}

for iter1 in range(dim-1):
	sys_safe |= {'((env1 = '+str(iter1)+') && (loc = '+str(iter1+ env_row*dim +1)+')) -> X !(loc = '+str(iter1 + env_row*dim)+')'}
	sys_safe |= {'((env1 = '+str(iter1+1)+') && (loc = '+str(iter1 + env_row*dim)+')) -> X !(loc = '+str(iter1 + env_row*dim +1)+')'}
	# if env_row>0 :
	# 	sys_safe |= {'((env1 = '+str(iter1)+') && (loc = '+str(iter1+ (env_row-1)*dim +1)+')) -> X !(loc = '+str(iter1 + env_row*dim +1)+')'}
	# 	sys_safe |= {'((env1 = '+str(iter1+1)+') && (loc = '+str(iter1+ (env_row-1)*dim)+')) -> X !(loc = '+str(iter1 + env_row*dim)+')'}
	# if env_row <dim-1 :
	# 	sys_safe |= {'((env1 = '+str(iter1)+') && (loc = '+str(iter1+ (env_row+1)*dim +1)+')) -> X !(loc = '+str(iter1 + env_row*dim +1)+')'}
	# 	sys_safe |= {'((env1 = '+str(iter1+1)+') && (loc = '+str(iter1+ (env_row+1)*dim)+')) -> X !(loc = '+str(iter1 + env_row*dim)+')'}

specs = spec.GRSpec(env_vars,sys_vars,env_init,sys_init,env_safe,sys_safe,env_prog,sys_prog) #GR(1) specification created
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
print ctrl
#print specs.pretty()
dumpsmach.write_python_case("rowEnvController.py", ctrl, classname="RowEnvCtrl")