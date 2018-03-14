from followMeController10_2_2 import FollowCtrl10_2_2
import time

controller = FollowCtrl10_2_2()

while True :
	env1 = int(raw_input("env1 = "))
	rel_row = int(raw_input("rel_row = "))
	rel_col = int(raw_input("rel_col = "))
	start = time.time()
	loc = controller.move(rel_row,env1,rel_col)
	end = time.time()
	print ("Current loc = ",loc,"diff time = ",(end-start))