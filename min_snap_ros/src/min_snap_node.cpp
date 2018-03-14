/*
"""
By using PyMASE, you agree to accept the following license agreement.

PyMASE (Beta Version) License Agreement

Contact: Ufuk 		Topcu				,  University of Texas at Austin (utopcu@utexas.edu).
		 Mohammed 	Alshiekh			,  University of Texas at Austin (malshiekh@utexas.edu)
		 Franck		Djeumou				,  Isae-Supaero					 (frmbouwe@gmail.com)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software must
display the following acknowledgment: "This product includes software developed
by Ufuk Topcu in the Department of Aerospace Engineering and Engineering 
Mechanics at the University of Texas at Austin".

4. Neither the name of the University of Texas at Austin nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF TEXAS AT AUSTIN AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF TEXAS AT AUSTIN OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
*/

#include <ros/ros.h>
#include "min_snap.h"
#include "qcontrol_defs/ConstraintAxis.h"
#include "qcontrol_defs/WayPoints.h"
#include "qcontrol_defs/SimplePathPlan.h"
#include "qcontrol_defs/PathPlan.h"
#include <string>
#include <iostream>

using namespace std;
using namespace Eigen;

bool min_snap_solve_inequality(qcontrol_defs::PathPlan::Request &req , qcontrol_defs::PathPlan::Response &res);
bool min_snap_solve_equality(qcontrol_defs::SimplePathPlan::Request &req , qcontrol_defs::SimplePathPlan::Response &res);
void transform_traj(const min_snap::Trajectory &traj, qcontrol_defs::Trajectory &traj_ros , bool x , bool y , bool z , bool yaw);
void transform_waypoint(const qcontrol_defs::WayPoints &waypoints_ros , min_snap::WayPoints &waypoints);
min_snap::ConstraintValue get_equality_constraint(double value , int derive_order);

int main(int argc, char **argv){

	//Node initialisation and node handle creation
	ros::init(argc,argv, "min_snap_node");
	ros::NodeHandle nh("~");

	ros::ServiceServer min_inequality = nh.advertiseService("/min_snap_trajectory_inequality", min_snap_solve_inequality);
	ros::ServiceServer min_equality = nh.advertiseService("/min_snap_trajectory", min_snap_solve_equality); 

	ROS_WARN("Use topic /min_snap_solve_inequality for constraints containing inequality ...");
	ROS_WARN("Use topic /min_snap_solve for constraints with only equality or corridors ...");

	ROS_INFO("min snap trajectory server launched ...");
	ros::spin();

	return 0;
}

bool min_snap_solve_inequality(qcontrol_defs::PathPlan::Request &req , qcontrol_defs::PathPlan::Response &res){
	//We try to compute a solution
	min_snap::solver_param params = DEFAULT_PARAMS;
	params.verbose = true;

	//Transform waypoints to a list of waypoint supported by the library
	min_snap::WayPoints waypoints;
	transform_waypoint(req.waypoints , waypoints);

	//Solve the problem
	ROS_INFO("Start Solving ....");
	min_snap::Solution sol(params , waypoints);
	ROS_INFO("Problem solved");

	if (! sol.has_solution()) {
		cout << "NO SOLUTION FOUND !! Check again your variables OR constraints ...." << endl;
		return false;
	}

	//save the solution in the response server message
	min_snap::Trajectory traj;
	traj.pva = vector<min_snap::PVA>();
	traj.wait_freq = vector<unsigned int>();
	sol.get_trajectory(req.freq , traj);

	//Copy in the ros message appropriate type
	transform_traj(traj , res.traj ,req.waypoints.x.size()> 1 , req.waypoints.y.size() > 1 , req.waypoints.z.size()> 1 , req.waypoints.yaw.size()>1);

	return true;

}

/*
*	Easy call to the service when the user only have equality constraints and corridors
*	The description of the message is pretty much explicit
*/
bool min_snap_solve_equality(qcontrol_defs::SimplePathPlan::Request &req , qcontrol_defs::SimplePathPlan::Response &res ){

	min_snap::solver_param params = DEFAULT_PARAMS;
	params.verbose = true;
	//Wrap everything in waypoint message that will be used by the Solution class to compute solution
	min_snap::WayPoints waypoints;

	waypoints.x = vector<min_snap::ConstraintAxis>();
	int iter =0;
	for(vector<double>::iterator x_ptr = req.x.begin() ; x_ptr != req.x.end() ; x_ptr++){
		min_snap::ConstraintAxis x_axis;
		x_axis.constraints = vector<min_snap::ConstraintValue>();
		x_axis.axis = *x_ptr;
		if(iter == 0){
			x_axis.constraints.push_back(get_equality_constraint(req.velx_init[0],1));
			x_axis.constraints.push_back(get_equality_constraint(req.accx_init[0],2));
			x_axis.constraints.push_back(get_equality_constraint(0,3));
			x_axis.constraints.push_back(get_equality_constraint(0,4));
		}
		if(iter == req.x.size() -1){
			x_axis.constraints.push_back(get_equality_constraint(req.velx_init[1],1));
			x_axis.constraints.push_back(get_equality_constraint(req.accx_init[1],2));
			x_axis.constraints.push_back(get_equality_constraint(0,3));
			x_axis.constraints.push_back(get_equality_constraint(0,4));
		}
		waypoints.x.push_back(x_axis);
		iter++;
	}

	waypoints.y = vector<min_snap::ConstraintAxis>();
	iter =0;
	for(vector<double>::iterator y_ptr = req.y.begin() ; y_ptr != req.y.end() ; y_ptr++){
		min_snap::ConstraintAxis y_axis;
		y_axis.axis = *y_ptr;
		y_axis.constraints = vector<min_snap::ConstraintValue>();
		if(iter == 0){
			y_axis.constraints.push_back(get_equality_constraint(req.vely_init[0],1));
			y_axis.constraints.push_back(get_equality_constraint(req.accy_init[0],2));
			y_axis.constraints.push_back(get_equality_constraint(0,3));
			y_axis.constraints.push_back(get_equality_constraint(0,4));
		}
		if(iter == req.y.size() -1){
			y_axis.constraints.push_back(get_equality_constraint(req.vely_init[1],1));
			y_axis.constraints.push_back(get_equality_constraint(req.accy_init[1],2));
			y_axis.constraints.push_back(get_equality_constraint(0,3));
			y_axis.constraints.push_back(get_equality_constraint(0,4));
		}
		iter++;
		waypoints.y.push_back(y_axis);
	}

	waypoints.z = vector<min_snap::ConstraintAxis>();
	iter=0;
	for(vector<double>::iterator z_ptr = req.z.begin() ; z_ptr != req.z.end() ; z_ptr++){
		min_snap::ConstraintAxis z_axis;
		z_axis.axis = *z_ptr;
		z_axis.constraints = vector<min_snap::ConstraintValue>();
		if(iter == 0){
			z_axis.constraints.push_back(get_equality_constraint(req.velz_init[0],1));
			z_axis.constraints.push_back(get_equality_constraint(req.accz_init[0],2));
			z_axis.constraints.push_back(get_equality_constraint(0,3));
			z_axis.constraints.push_back(get_equality_constraint(0,4));
		}
		if(iter == req.z.size() -1){
			z_axis.constraints.push_back(get_equality_constraint(req.velz_init[1],1));
			z_axis.constraints.push_back(get_equality_constraint(req.accz_init[1],2));
			z_axis.constraints.push_back(get_equality_constraint(0,3));
			z_axis.constraints.push_back(get_equality_constraint(0,4));
		}
		iter++;
		waypoints.z.push_back(z_axis);
	}

	waypoints.yaw = vector<min_snap::ConstraintAxis>();
	iter=0;
	for(vector<double>::iterator yaw_ptr = req.yaw.begin() ; yaw_ptr != req.yaw.end() ; yaw_ptr++){
		min_snap::ConstraintAxis yaw_axis;
		yaw_axis.axis = *yaw_ptr;
		yaw_axis.constraints = vector<min_snap::ConstraintValue>();
		if(iter == 0){
			yaw_axis.constraints.push_back(get_equality_constraint(req.yaw_rate_init[0],1));
			yaw_axis.constraints.push_back(get_equality_constraint(0,2));
		}
		if(iter == req.z.size() -1){
			yaw_axis.constraints.push_back(get_equality_constraint(req.yaw_rate_init[1],1));
			yaw_axis.constraints.push_back(get_equality_constraint(0,2));
		}
		iter++;
		waypoints.yaw.push_back(yaw_axis);
	}

	//Fill in the corridors list
	waypoints.corridors = req.corridors;

	//Fill in the time list
	waypoints.t = req.t;

	//Try to compute a solution
	ROS_INFO("Start Solving ....");
	min_snap::Solution sol(params , waypoints);
	ROS_INFO("Problem solved !!!");
	if (! sol.has_solution()) {
		cout << "NO SOLUTION FOUND !! Check again your variables OR constraints ...." << endl;
		return false;
	}

	//save the solution in the response server message
	min_snap::Trajectory traj;
	traj.pva = vector<min_snap::PVA>();
	traj.wait_freq = vector<unsigned int>();
	sol.get_trajectory(req.freq , traj);

	//Transform it back
	transform_traj(traj , res.traj , req.x.size()> 1 , req.y.size() > 1 , req.z.size()> 1 , req.yaw.size()>1);

	return true;

}

min_snap::ConstraintValue get_equality_constraint(double value , int derive_order){
	min_snap::ConstraintValue curr_cont;
	curr_cont.bndl = value;
	curr_cont.bndu = value;
	curr_cont.bndl_valid = true;
	curr_cont.bndu_valid = true;
	curr_cont.derive_order = derive_order;
	return curr_cont;
}

void transform_traj(const min_snap::Trajectory &traj, qcontrol_defs::Trajectory &traj_ros , bool x , bool y , bool z , bool yaw){
	for(vector<min_snap::PVA>::const_iterator pva_iter = traj.pva.begin(); pva_iter != traj.pva.end() ;pva_iter++){
		qcontrol_defs::PVA curr_pva;
		if( x ){
			curr_pva.pos.x = pva_iter->pos(0);
			curr_pva.vel.x = pva_iter->vel(0);
			curr_pva.acc.x = pva_iter->acc(0);
		}
		if( y ){
			curr_pva.pos.y = pva_iter->pos(1);
			curr_pva.vel.y = pva_iter->vel(1);
			curr_pva.acc.y = pva_iter->acc(1);
		}
		if( z ){
			curr_pva.pos.z = pva_iter->pos(2);
			curr_pva.vel.z = pva_iter->vel(2);
			curr_pva.acc.z = pva_iter->acc(2);
		}
		if( yaw ){
			curr_pva.yaw = pva_iter->yaw;
			curr_pva.yaw_rate = pva_iter->yaw_rate;
		}
		traj_ros.pva.push_back(curr_pva);
	}
	traj_ros.wait_freq = traj.wait_freq;
}

void transform_cont_axis(const qcontrol_defs::ConstraintAxis &axis , min_snap::ConstraintAxis const_min_snap){
	const_min_snap.axis = axis.axis;
	const_min_snap.constraints = vector<min_snap::ConstraintValue>();
	for(vector<qcontrol_defs::ConstraintValue>::const_iterator const_val = axis.constraints.begin() ; const_val != axis.constraints.end() ; const_val++){
		min_snap::ConstraintValue curr_const;
		curr_const.bndl = const_val->bndl;
		curr_const.bndu = const_val->bndu;
		curr_const.bndl_valid = const_val->bndl_valid;
		curr_const.bndu_valid = const_val->bndu_valid;
		curr_const.derive_order = const_val->derive_order;
		const_min_snap.constraints.push_back(curr_const);
	}
}

void transform_waypoint(const qcontrol_defs::WayPoints &waypoints_ros , min_snap::WayPoints &waypoints){
	waypoints.x = vector<min_snap::ConstraintAxis>();
	for(vector<qcontrol_defs::ConstraintAxis>::const_iterator axis= waypoints_ros.x.begin(); axis != waypoints_ros.x.end() ; axis++){
		min_snap::ConstraintAxis curr_axis;
		transform_cont_axis(*axis,curr_axis);
		waypoints.x.push_back(curr_axis);
	}

	waypoints.y = vector<min_snap::ConstraintAxis>();
	for(vector<qcontrol_defs::ConstraintAxis>::const_iterator axis= waypoints_ros.y.begin(); axis != waypoints_ros.y.end() ; axis++){
		min_snap::ConstraintAxis curr_axis;
		transform_cont_axis(*axis,curr_axis);
		waypoints.y.push_back(curr_axis);
	}

	waypoints.z = vector<min_snap::ConstraintAxis>();
	for(vector<qcontrol_defs::ConstraintAxis>::const_iterator axis= waypoints_ros.z.begin(); axis != waypoints_ros.z.end() ; axis++){
		min_snap::ConstraintAxis curr_axis;
		transform_cont_axis(*axis,curr_axis);
		waypoints.z.push_back(curr_axis);
	}

	waypoints.yaw = vector<min_snap::ConstraintAxis>();
	for(vector<qcontrol_defs::ConstraintAxis>::const_iterator axis= waypoints_ros.yaw.begin(); axis != waypoints_ros.x.end() ; axis++){
		min_snap::ConstraintAxis curr_axis;
		transform_cont_axis(*axis,curr_axis);
		waypoints.yaw.push_back(curr_axis);
	}

	waypoints.corridors = waypoints_ros.corridors;
	waypoints.t = waypoints_ros.t;
}