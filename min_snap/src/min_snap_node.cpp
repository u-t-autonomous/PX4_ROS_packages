#include <ros/ros.h>
#include "min_snap_node.h"
#include "min_snap/ConstraintAxis.h"
#include "min_snap/WayPoints.h"
#include "min_snap/PathPlan.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <iostream>

using namespace std;
using namespace Eigen;

//Global variable
solver_param params = DEFAULT_PARAMS;

bool min_snap_solve(min_snap::PathPlan::Request &req , min_snap::PathPlan::Response &res){
	if(req.waypoints.x.size()<=1 && req.waypoints.y.size() <=1 && req.waypoints.z.size() <=1 && req.waypoints.yaw.size() <=1){
		ROS_ERROR("X and Y and Z and Yaw have not enough points .... ");
	}
	string axis_label[MAX_DIM];
	for(int i = 0 ; i<MAX_DIM ; i++){
		axis_label[i] = "";
	}
	int d =0;

	MatrixXd x(1,1);
	MatrixXd yaw_vect(1,1);
	vector<vector<min_snap::ConstraintValue> > constraints;
	vector<vector<min_snap::ConstraintValue> > constraints_yaw;
	int problem_size = req.waypoints.x.size()>1 ?req.waypoints.x.size():(req.waypoints.y.size()>1 ? req.waypoints.y.size() :(req.waypoints.z.size()>1? req.waypoints.z.size():req.waypoints.yaw.size()));
	for(int i=0; i<problem_size;i++){
		constraints.push_back(vector<min_snap::ConstraintValue>());
		constraints_yaw.push_back(vector<min_snap::ConstraintValue>());
	}

	if(req.waypoints.x.size()> 0){
		x.conservativeResize(req.waypoints.x.size(),d+1);
		int iter = 0;
		for (vector<min_snap::ConstraintAxis>::iterator x_iter =req.waypoints.x.begin(); x_iter!=req.waypoints.x.end();x_iter++){
			x(iter,d) = x_iter->axis;
			for(vector<min_snap::ConstraintValue>::iterator const_val = (x_iter->constraints).begin();const_val!=(x_iter->constraints).end();const_val++){
				const_val->label = d;
				constraints[iter].push_back(*const_val);
			}
			iter++;
		}
		axis_label[d] = "x";
		d++;
	}
	if(req.waypoints.y.size()> 1){
		if(x.rows() != 0 && x.rows() != req.waypoints.y.size()){
			ROS_ERROR("X and Y list points have differents size ... ");
			return false;
		}
		x.conservativeResize(req.waypoints.y.size(),d+1);
		int iter = 0;
		for (vector<min_snap::ConstraintAxis>::iterator y_iter =req.waypoints.y.begin(); y_iter!=req.waypoints.y.end();y_iter++){
			x(iter,d) = y_iter->axis;
			for(vector<min_snap::ConstraintValue>::iterator const_val = (y_iter->constraints).begin();const_val!=(y_iter->constraints).end();const_val++){
				const_val->label = d;
				constraints[iter].push_back(*const_val);
			}
			iter++;
		}
		axis_label[d] = "y";
		d++;
	}
	if(req.waypoints.z.size() >1){
		if(x.rows() != 0 && x.rows() != req.waypoints.z.size()){
			ROS_ERROR("Z list has different size from X OR Y ... ");
			return false;
		}
		x.conservativeResize(req.waypoints.z.size(),d+1);
		int iter = 0;
		for (vector<min_snap::ConstraintAxis>::iterator z_iter =req.waypoints.z.begin(); z_iter!=req.waypoints.z.end();z_iter++){
			x(iter,d) = z_iter->axis;
			for(vector<min_snap::ConstraintValue>::iterator const_val = (z_iter->constraints).begin();const_val!=(z_iter->constraints).end();const_val++){
				const_val->label = d;
				constraints[iter].push_back(*const_val);
			}
			iter++;
		}
		axis_label[d] = "z";
		d++;
	}

	//If yaw trajectory is asked
	if(req.waypoints.yaw.size() >1){
		if(x.rows() > 1 && x.rows() != req.waypoints.yaw.size() ){
			ROS_ERROR("Yaw list has different size from X OR Y OR Z ... ");
			return false;
		}
		yaw_vect = MatrixXd(req.waypoints.yaw.size(),1);
		int iter = 0;
		for (vector<min_snap::ConstraintAxis>::iterator yaw_iter =req.waypoints.yaw.begin(); yaw_iter!=req.waypoints.yaw.end();yaw_iter++){
			yaw_vect(iter,0) = yaw_iter->axis;
			for(vector<min_snap::ConstraintValue>::iterator const_val = (yaw_iter->constraints).begin();const_val!=(yaw_iter->constraints).end();const_val++){
				const_val->label = d;
				constraints_yaw[iter].push_back(*const_val);
			}
			iter++;
		}
		axis_label[d] ="yaw";
	}
	
	//Corridors constraints
	VectorXd corridors = VectorXd::Zero(x.rows()> 1 ? x.rows()-1 : yaw_vect.rows()-1);
	int iter1 = 0;
	for (vector<double>::iterator cor_iter = req.waypoints.corridors.begin(); cor_iter!=req.waypoints.corridors.end();cor_iter++){
		corridors(iter1) = *cor_iter;
		iter1++;
	}

	//Time input
	double t_init , t_end;
	bool optimal_time = false;
	VectorXd t(x.rows()> 1 ? x.rows() : yaw_vect.rows());

	if (req.waypoints.t.size() == 2){
		optimal_time = true;
		t_init = req.waypoints.t[0];
		t_end = req.waypoints.t[1];
	} else {
		if((x.rows() > 1 && req.waypoints.t.size()!= x.rows()) ||(yaw_vect.rows() > 1 && req.waypoints.t.size()!= yaw_vect.rows())){
			ROS_ERROR("Time list has different size from X OR Y OR Z OR YAW ... ");
			return false;
		}
		int iter_t =0;
		t_init = req.waypoints.t[0];
		t_end = req.waypoints.t[req.waypoints.t.size()-1];
		for(vector<double>::iterator t_iter = req.waypoints.t.begin(); t_iter!=req.waypoints.t.end();t_iter++){
			t(iter_t) = *t_iter;
			iter_t++;
		}
	}

	//Now we solve the problem
	Solution solution(params.N,(x.rows()> 1 ? x.rows()-1 : yaw_vect.rows()-1),d);
	for(int i = 0 ; i< MAX_DIM ; i++){
		solution.axis_label[i] = axis_label[i];
	}

	if(optimal_time){
		if(x.rows() > 1){	
			compute_min_snap_optimal_segment_times(params,x,0.0,1.0,constraints,corridors,solution.sol_point,solution.t);
			if(yaw_vect.rows() > 1){
				solver_param params_yaw = params;
				params_yaw.derive_order = SNAP_YAW_ORDER;
				double cost;
				compute_min_snap_default(params_yaw,yaw_vect,solution.t,constraints_yaw,VectorXd::Zero(yaw_vect.rows()-1),solution.yaw,cost);
			}
			solution.t *= (t_end-t_init);
			solution.t += VectorXd::Constant(x.rows(),t_init);
		}else {
			solver_param params_yaw = params;
			params_yaw.derive_order = SNAP_YAW_ORDER;
			compute_min_snap_optimal_segment_times(params_yaw,yaw_vect,0.0,1.0,constraints_yaw,VectorXd::Zero(yaw_vect.rows()-1),solution.yaw,solution.t);
			solution.t *= (t_end-t_init);
			solution.t += VectorXd::Constant(yaw_vect.rows(),t_init);
		}
	}else {
		double cost;
		solution.t = t;
		t -= VectorXd::Constant(t.size(),t_init);
		t*=(1.0/(t_end-t_init));
		if(x.rows() > 1){	
			compute_min_snap_default(params,x,t,constraints,corridors,solution.sol_point,cost);
		}
		if (yaw_vect.rows() > 1){
			solver_param params_yaw = params;
			params_yaw.derive_order = SNAP_YAW_ORDER;
			compute_min_snap_default(params_yaw,yaw_vect,t,constraints_yaw,VectorXd::Zero(yaw_vect.rows()-1),solution.yaw,cost);
		}
	}

	if(x.rows()>1){
		cout << x <<endl;
	}
	if(yaw_vect.cols()>1){
		cout << yaw_vect <<endl;
	}
	cout << corridors << endl;
	cout << solution.t << endl;

	solution.get_trajectory(req.freq,res.traj);
	return true;
}

int main(int argc, char **argv){
	//Node initialisation and node handle creation
	ros::init(argc,argv, "min_snap_node");
	ros::NodeHandle nh("~");
	ros::ServiceServer command_service = nh.advertiseService("/min_snap_trajectory", min_snap_solve);
	ROS_INFO("min snap trajectory server launched ...");
	ros::spin();
}

Solution::Solution(int m_N,int m_M, int m_d ){
	N = m_N;
	M = m_M;
	d = m_d;
	sol_point = VectorXd(M*N*d);
	yaw = VectorXd(M*N);
	t = VectorXd(M+1);
	for(int i=0 ; i<MAX_DIM;i++){
		axis_label[i] = "";
	}
}

void Solution::get_solution(double m_t, int current_seg, geometry_msgs::Point &position,geometry_msgs::Point &velocity,geometry_msgs::Point &acceleration , double &m_yaw , double &m_yaw_rate){
	MatrixXd derivatives(3,N);
	generate_derivatives(m_t,derivatives);
	for(int i = 0; i< d ; i++){
		VectorXd label_d = derivatives * (sol_point.segment(i*M*N + current_seg*N,N));
		if(axis_label[i] == "x"){
			position.x = label_d(0);
			velocity.x = label_d(1);
			acceleration.x = label_d(2);
		}else if(axis_label[i] == "y"){
			position.y = label_d(0);
			velocity.y = label_d(1);
			acceleration.y = label_d(2);
		}else if(axis_label[i] == "z"){
			position.z = label_d(0);
			velocity.z = label_d(1);
			acceleration.z = label_d(2);
		}
		if(axis_label[d] == "yaw"){
			VectorXd yaw_point = derivatives.block(0,0,2,N) * yaw.segment(current_seg*N,N);
			m_yaw = yaw_point(0);
			m_yaw_rate = yaw_point(1);
		}
	}
}

void Solution::get_trajectory(int freq , min_snap::Trajectory &traj){
	double t_init = t(0);
	double t_final = t(M);
	double time_factor = t_final - t_init;
	const double scale_freq = (1.0/(freq*time_factor)); 
	double current_t = scale_freq;
	int current_seg = 0;
	bool has_yaw = false;
	//Insert initial condition
	geometry_msgs::Point pos_,vel_,acc_;
	double yaw_ = MAX_REAL_VALUE;
	double yaw_rate_ = MAX_REAL_VALUE;
	this->get_solution(0.0,0,pos_,vel_,acc_,yaw_,yaw_rate_);

	traj.position.push_back(pos_);
	traj.velocity.push_back(vel_);
	traj.acceleration.push_back(acc_);
	if (yaw_ != MAX_REAL_VALUE){
		has_yaw = true;
		traj.yaw.push_back(yaw_);
		traj.yaw_rate.push_back(yaw_rate_);
	}
	traj.wait_freq.push_back(freq);

	//for all the point between t_init and t_end excluding t_end at freq
	while(current_t < 1.0){
		this->get_solution(current_t,current_seg,pos_,vel_,acc_,yaw_,yaw_rate_);
		if(has_yaw){
			traj.yaw.push_back(yaw_);
			traj.yaw_rate.push_back(yaw_rate_);
		}
		traj.position.push_back(pos_);
		traj.velocity.push_back(vel_);
		traj.acceleration.push_back(acc_);
		traj.wait_freq.push_back(freq);
		current_t += scale_freq;
		if(current_t> (t(current_seg+1)-t_init)/time_factor){
			current_seg++;
		}
	}
	//We add the freq to reach the final point
	this->get_solution(1.0,M-1,pos_,vel_,acc_,yaw_,yaw_rate_);
	if(has_yaw){
		traj.yaw.push_back(yaw_);
		traj.yaw_rate.push_back(yaw_rate_);
	}
	traj.position.push_back(pos_);
	traj.velocity.push_back(vel_);
	traj.acceleration.push_back(acc_);
	traj.wait_freq.push_back(1.0/((1.0 - current_t + scale_freq)*time_factor));
}