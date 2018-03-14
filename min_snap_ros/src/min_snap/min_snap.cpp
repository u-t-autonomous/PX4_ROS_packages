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

#include <omp.h>
#include "min_snap.h"
#include <Eigen/SPQRSupport>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_poly.h>
#include <iostream>
#include <stdlib.h>
#include <functional>
#include <ctime>

using namespace std;
using namespace Eigen;

using namespace min_snap;

/*
* Custom structure to pass to function evaluation when solving the problem with inequality using
* the logarithm barrier method algorithm as penalty when approaching bounds
*/
struct barrier_params {
	SPQR<SparseMatrix<double> > *solver;
	SparseMatrix<double> *H_matrix;
	VectorXd *X;
	bnd_interval *arr_ind_ineq;
	double h;
	double barrier_cost;
	double mu0;
	double scale_factor;
};

void generate_pow(double t, VectorXd &out);

void generate_derivatives(double t , MatrixXd &out);

void generate_cost_matrix_1d(double min_seg_dur,int N, int derive_order,const VectorXd &t, vector<T> &H , MatrixXd M[]);

bool generate_deriv_order_constraints(int N,int derive_order , const MatrixXd &x,const new_time_scale &time_scale,
											const vector< ConstraintValue> constraints[],
												vector<VectorXd> &A, vector<bnd_interval> &b , const MatrixXd M_inv[]);

bool generate_corridors(int N , int nc , const MatrixXd &x, const VectorXd &t,const VectorXd &corridors,
								vector<T> &A, vector<bnd_interval> &b , const MatrixXd M_inv[]);

double lagrange_solve(bool return_cost, const solver_param &params , const double scale_fator,
							const int M , const int dim_, const vector<T> &H ,
								 const vector<T> &A , const vector<bnd_interval> &b ,VectorXd &sol);

double compute_min_snap_with_inequality(bool return_cost, const solver_param &params, const double scale_factor ,const int M, 
											const int dim_, const std::vector<T> &H , const vector<T> &A, 
													const vector<bnd_interval> &b , VectorXd &sol);

bool gradient_descent_minsnap_optimal(const solver_param &params, const MatrixXd &x,double t_init,double t_end,
											const vector<ConstraintValue> constraints[],
												const VectorXd &corridors, VectorXd &sol_x, new_time_scale &time_scale);

/*
* Check if the problem has a solution
* return true if a solution has been found and false if not
*/
bool Solution::has_solution(){
	return solve_succeed;
}

/*
* Solution constructor which solve the minimization problem
* check attribute solve_succeed to see if a solution has been found
*/
Solution::Solution(const solver_param params , const WayPoints &waypoints){

	if(waypoints.x.size()<=1 && waypoints.y.size() <=1 && waypoints.z.size() <=1 && waypoints.yaw.size() <=1){
		cout << "Waypoints list size is less than ONE !!!! check the waypoint list again ... " << endl;
		solve_succeed = false;
		return;
	}

	//Initialization of the axis label which will be use to associate x solution to the appropriate axis
	for(int i = 0 ; i<MAX_DIM ; i++){
		axis_label[i] = "";
	}

	//We count the number of axis gien by user
	int d =0;

	MatrixXd x(1,1), yaw_vect(1,1); 												//Initialization with a size of one

	int problem_size = waypoints.x.size()>1 ?waypoints.x.size():(waypoints.y.size()>1 ? waypoints.y.size() :(waypoints.z.size()>1? waypoints.z.size():waypoints.yaw.size()));
	
	vector<ConstraintValue> constraints[problem_size]; 								//Constraints for x,y,z waypoints
	vector<ConstraintValue> constraints_yaw[problem_size];							//Constraints for yaw waypoints 

	//Check the number of point in the waypoint list
	
	//Initialize constraints to no constraints
	for(int i=0; i<problem_size;i++){
		constraints[i] = vector<ConstraintValue>();
		constraints_yaw[i] = vector<ConstraintValue>();
	}

	//Check if x has a waypoint list
	if(waypoints.x.size()> 0){
		x.conservativeResize(waypoints.x.size(),d+1);								//Resize the x matrix with the new axis information
		int iter = 0;
		for (vector<ConstraintAxis>::const_iterator x_iter =waypoints.x.begin(); x_iter!=waypoints.x.end();x_iter++){
			x(iter,d) = x_iter->axis;												//axis contains the current setpoint
			//For all constraints applied to this setpoint, we label it --> Now x will have label 0 if there exist any setpoint on x axis
			for(vector<ConstraintValue>::const_iterator const_val = (x_iter->constraints).begin();const_val!=(x_iter->constraints).end();const_val++){
				ConstraintValue new_const = *const_val;
				new_const.label = d;
				constraints[iter].push_back(new_const);
			}
			iter++;
		}
		axis_label[d] = "x";											//Set the label in order to return a proper solution
		d++;
	}

	//Check if y has a waypoint list and do the same operations as we did if x existed
	if(waypoints.y.size()> 1){
		//Check if x and y have the same size in case x has a waypoint list
		if(x.rows() != 0 && x.rows() != waypoints.y.size()){
			cout << "X and Y have not the same waypoint list size ... " << endl;
			solve_succeed = false;
			return;
		}
		x.conservativeResize(waypoints.y.size(),d+1);
		int iter = 0;
		for (vector<ConstraintAxis>::const_iterator y_iter =waypoints.y.begin(); y_iter!=waypoints.y.end();y_iter++){
			x(iter,d) = y_iter->axis;
			for(vector<ConstraintValue>::const_iterator const_val = (y_iter->constraints).begin();const_val!=(y_iter->constraints).end();const_val++){
				ConstraintValue new_const = *const_val;
				new_const.label = d;
				constraints[iter].push_back(new_const);
			}
			iter++;
		}
		axis_label[d] = "y";
		d++;
	}

	//Check if y has a waypoint list and do the same operations as we did if x // y existed
	if(waypoints.z.size() >1){
		if(x.rows() != 0 && x.rows() != waypoints.z.size()){
			cout << "Z have not the same waypoint list size as X or Y ... " << endl;
			solve_succeed =  false;
			return;
		}
		x.conservativeResize(waypoints.z.size(),d+1);
		int iter = 0;
		for (vector<ConstraintAxis>::const_iterator z_iter =waypoints.z.begin(); z_iter!=waypoints.z.end();z_iter++){
			x(iter,d) = z_iter->axis;
			for(vector<ConstraintValue>::const_iterator const_val = (z_iter->constraints).begin();const_val!=(z_iter->constraints).end();const_val++){
				ConstraintValue new_const = *const_val;
				new_const.label = d;
				constraints[iter].push_back(new_const);
			}
			iter++;
		}
		axis_label[d] = "z";
		d++;
	}

	//If yaw trajectory is asked create the matrix of yaw setpoints and it constraints
	if(waypoints.yaw.size() >1){
		if(x.rows() > 1 && x.rows() != waypoints.yaw.size() ){
			cout << "Yaw list has different size from X OR Y OR Z ... This can not be solve since they used the same time list " << endl;
			solve_succeed =  false;
			return ;
		}
		yaw_vect = MatrixXd(waypoints.yaw.size(),1);
		int iter = 0;
		for (vector<ConstraintAxis>::const_iterator yaw_iter =waypoints.yaw.begin(); yaw_iter!=waypoints.yaw.end();yaw_iter++){
			yaw_vect(iter,0) = yaw_iter->axis;
			for(vector<ConstraintValue>::const_iterator const_val = (yaw_iter->constraints).begin();const_val!=(yaw_iter->constraints).end();const_val++){
				ConstraintValue new_const = *const_val;
				new_const.label = d;
				constraints_yaw[iter].push_back(new_const);
			}
			iter++;
		}
		axis_label[d] ="yaw";
	}
	
	//Setting corridors constraints
	VectorXd corridors = VectorXd::Zero(problem_size-1);
	int iter1 = 0;
	bool contain_corridors = false;
	for (vector<double>::const_iterator cor_iter = waypoints.corridors.begin(); cor_iter!=waypoints.corridors.end();cor_iter++){
		corridors(iter1) = *cor_iter;
		if(corridors(iter1) >0){
			contain_corridors = true;
		}
		iter1++;
	}

	//Setting Time input
	double t_init;
	double t_end;
	t_scale.t = VectorXd(problem_size);												//Set the time attribute --> Ensure that time start at 0 for implementation respect
	bool optimal_time = false;											

	if (waypoints.t.size() == 2 && problem_size !=2 ){
		t_init = waypoints.t[0];
		t_end = waypoints.t[1];
		trajectory_duration = t_end - t_init;
		optimal_time = true;
	} else {
		if(problem_size !=  waypoints.t.size()){
			cout << "Time list has different size from X OR Y OR Z OR YAW ... " << endl;
			solve_succeed =  false;
			return ;
		}
		int iter = 0;
		for(vector<double>::const_iterator t_iter = waypoints.t.begin() ; t_iter != waypoints.t.end() ; t_iter++){
			t_scale.t(iter) = (*t_iter);
			iter++;
		}
		t_scale.scale_factor = 1.0;
		t_scale.t_init = t_scale.t(0);
		t_init = t_scale.t(0);
		t_end = t_scale.t(problem_size-1);
		trajectory_duration = t_end - t_init;
	}

	//Setting attribute number of segment and number of axis (Not counting yaw)
	//solve_succeed = true;
	M = problem_size - 1;
	axis_number = x.cols();
	solution = VectorXd(M * (SNAP_POSITION_ORDER+1) * 2 * axis_number);
	yaw = VectorXd(M * (SNAP_YAW_ORDER + 1) * 2);

	if(params.verbose){
		cout << "Init Time = " << t_init << " --- End Time  = "<< t_end << endl;
		cout << "variables used for solver  = " << d << endl;
		cout << "Axis are : ";
		for(int i = 0 ; i<=d;i++){
			cout << axis_label[i] << " ___ ";
		}
		cout << endl;
		if(axis_number> 0 ){	
			cout << "-----------------   WayPoints list  -------------------- " <<endl;
			cout << x.transpose() << "\n" << endl;
		}
		if(axis_label[d] == "yaw"){
			cout << "-----------------   Yaw list  -------------------- " <<endl;
			cout << yaw_vect.transpose() << "\n" << endl;
		}
		if(contain_corridors){
			cout << "-----------------   corridors list  -------------------- " <<endl;
			cout << corridors.transpose() << "\n" << endl;
		}

		if (! optimal_time){
			cout << "-----------------   Time list (Translated by T_init)  -------------------- " <<endl;
			cout << t_scale.t.transpose() << endl;
			cout << "Scale factor = " << t_scale.scale_factor << "\n" << endl;
		}else {
			cout << "Optimal time segment asked ...." << "\n" << endl;
		}

		for(int i = 0 ; i< problem_size ; i++){
			for (vector<ConstraintValue>::iterator const_value  = constraints[i].begin() ; const_value != constraints[i].end() ; const_value++){
				cout << "label = " << const_value->label << " , Order_derivate = " << const_value->derive_order << " , bndl = " << const_value->bndl << " , bndl_valid = " << const_value->bndl_valid ;
				cout << " , bndu = " << const_value->bndu << " , bndu_valid = " << const_value ->bndu_valid << "\n" << endl;
			}
		}
	}

	if(optimal_time){
		if(x.rows() > 1){	
			solver_param params_pos = params;
			params_pos.derive_order = SNAP_POSITION_ORDER;
			if(! contain_corridors ){
				if(! compute_min_snap_optimal_segment_times(params_pos,x,t_init,t_end,constraints,corridors,solution,t_scale)){
					solve_succeed = false;
					return ;
				}
			} else {
				t_scale.t(problem_size-1) = t_end;
				t_scale.t(0) = t_init;
				if(! compute_min_snap_iter_corridors(true,params_pos,x,t_scale,constraints,corridors,solution)){
					solve_succeed = false;
					return ;
				}
			}
			if(axis_label[d] == "yaw"){
				solver_param params_yaw = params;
				params_yaw.derive_order = SNAP_YAW_ORDER;
				if(! compute_min_snap_default(params_yaw,yaw_vect,t_scale,constraints_yaw,VectorXd::Zero(yaw_vect.rows()-1),yaw)){
					solve_succeed = false;
					return ;
				}
			}
		}else {
			solver_param params_yaw = params;
			params_yaw.derive_order = SNAP_YAW_ORDER;
			if(! compute_min_snap_optimal_segment_times(params_yaw,yaw_vect,t_init,t_end,constraints_yaw,VectorXd::Zero(yaw_vect.rows()-1),yaw,t_scale)){
				solve_succeed = false;
				return ;
			}
		}
	}else {
		if(x.rows() > 1){
			solver_param params_pos = params;
			params_pos.derive_order = SNAP_POSITION_ORDER;
			if(!contain_corridors){
				if(! compute_min_snap_default(params_pos,x,t_scale,constraints,corridors,solution)){
					solve_succeed = false;
					return ;
				}
			} else{
				if(! compute_min_snap_iter_corridors(false,params_pos,x,t_scale,constraints,corridors,solution)){
					solve_succeed = false;
					return ;
				}
			}
		}
		if (axis_label[d] == "yaw"){
			solver_param params_yaw = params;
			params_yaw.derive_order = SNAP_YAW_ORDER;
			if(! compute_min_snap_default(params_yaw,yaw_vect,t_scale,constraints_yaw,VectorXd::Zero(yaw_vect.rows()-1),yaw)){
				solve_succeed = false;
				return ;
			}
		}
	}

	M = x.rows() - 1;

	solve_succeed = true;
}

/*
*	User friendly function to get a solution at a specified m_t time
*/
void Solution::get_solution(double m_t , PVA &pva){
	int current_seg = 0;
	if(m_t - t_scale.t(0) < 0){
		cout << "Not a valid time ! Must be >= t_init ..." << endl;
	}else{
		for(int i=0; i< M ; i++){
			if(m_t - t_scale.t(i)>= 0 && m_t - t_scale.t(i) < t_scale.t(i+1)){
				current_seg = i;
			}
		}
	}
	get_solution(m_t , current_seg , pva);
}

/*
* Private method that return the PVA and yaw value for a time m_t on a specify segment
* current_seg is the index of the current segment
* Results are store in the PVA message argument pva
*/
void Solution::get_solution(double m_t, int current_seg, PVA &pva){
	MatrixXd derivatives(3,N_pos);
	generate_derivatives(m_t - t_scale.t(current_seg),derivatives);
	VectorXd label_d = VectorXd::Zero(3);
	VectorXd yaw_point(2);
	for(int i = 0; i< axis_number ; i++){
		 label_d = derivatives * (solution.segment(i*M*N_pos + current_seg*N_pos,N_pos));
		 //cout << derivatives << endl;
		 //cout << (solution.segment(i*M*N_pos + current_seg*N_pos,N_pos)).transpose() << endl;
		if(axis_label[i] == "x"){
			pva.pos(0) = label_d(0);
			pva.vel(0) = label_d(1) * t_scale.scale_factor;
			pva.acc(0) = label_d(2) * t_scale.scale_factor * t_scale.scale_factor;
		}else if(axis_label[i] == "y"){
			pva.pos(1) = label_d(0);
			pva.vel(1) = label_d(1) * t_scale.scale_factor;
			pva.acc(1) = label_d(2) * t_scale.scale_factor * t_scale.scale_factor;
		}else if(axis_label[i] == "z"){
			pva.pos(2) = label_d(0);
			pva.vel(2) = label_d(1) * t_scale.scale_factor;
			pva.acc(2) = label_d(2) * t_scale.scale_factor * t_scale.scale_factor;
		}
		if(axis_label[axis_number] == "yaw"){
			yaw_point = derivatives.block(0,0,2,N_yaw) * yaw.segment(current_seg*N_yaw,N_yaw);
			pva.yaw = yaw_point(0);
			pva.yaw_rate = yaw_point(1) * t_scale.scale_factor;
		}
	}
}

/*
* Main function that will be use by user after successful creation of a Solution object
* This function store in traj all the PVA for the complete trajectory at the specify frequency
*/
bool Solution::get_trajectory(int freq , Trajectory &traj){
	if(! solve_succeed){
		return false;
	}

	//double scale_t_end =  t_scale.t(t_scale.t.size()-1);

	const double scale_delta_t = (t_scale.scale_factor/freq); 
	double current_t = 0;
	int current_seg = 0;

	//Insert initial condition
	PVA pva;
	//this->get_solution(0.0,0,pva);
	//traj.pva.push_back(pva);
	// /traj.wait_freq.push_back(freq);

	//int nb_point = (int) ((scale_t_end/t_scale.scale_factor) * freq);
	int nb_point = (int) (trajectory_duration * freq);
	//cout << " nb_point " << nb_point << "  t_end = " << scale_t_end << "scale_factor = " << t_scale.scale_factor << endl;
	//cout << "Intial time : " << (scale_t_end/t_scale.scale_factor) <<  " nb_point = " << nb_point << endl;
	//#pragma omp for ordered schedule(static)
	for( int i = 0 ; i< nb_point ; i++){
		this->get_solution(i*scale_delta_t ,current_seg,pva);
		traj.pva.push_back(pva);
		traj.wait_freq.push_back(freq);
		current_t += scale_delta_t;
		if(current_t > t_scale.t(current_seg+1)){
			current_seg++;
		}
	}
	return true;
}

/*
* generate and store inside out 1 t t^2 t^3 ........ t^n 
* with n = out.size()
*/
void generate_pow(double t , VectorXd &out){
	out(0) = 1;
	for(int i=0; i <out.size()-1;i++){
		out(i+1) = out(i)*t;
	}
}

/*
* Generate matrix of derivatives
* For example if out.rows() = 2  and out.cols() =  n then 
* res = [ 1 t t^2 ....... t^n]
*		[ 0 1 2t .........nt^n-1]
* It stops at the out.cols()-1 derivatives and store the result in out
*/
void generate_derivatives(double t, MatrixXd &out){
	VectorXd row_0(out.cols());
	generate_pow(t,row_0);
	out.row(0) =  row_0;
	if(out.rows() <= 1){
		return ;
	}
	for (int i =0; i< out.cols() ; i++){
		int h = 1;
		for (int j = 1 ; j< out.rows() ; j++){
			if(i<j){
				out(j,i) = 0; 
				continue; 
			}
			h *= (i-j+1);
			out(j,i) = h*out(0,i-j);
		}
	}
}

/**
* this function saves in H the cost matrix for only one axis
* In the new problem formulation :
* 			H_new  = t(inv(M)) * H * inv(M)
* Where M is made in a way :
*			x_new = M * x    ---> x_new = t(p0 v0 a0 j0 s0 p1 v1 a1 j1 s1) --> Go untill we reach the deriv order (This is just an example of notation for snap order)
* p = position , v = velocity , a = acceleration , j = jerk derv , s = snap der
* So basically :
*				M = [generate_derivatives(0 , derive_order , N ).generate_derivatives(duration , derive_order , N)]
*			
* Note that the global cost matrix a Diagonal Matrix with this generated 1d cost matrix as diagonal block.
* Basically 
*					Hs(x|y|z|yaw) (i,j) =  Hs(x|y|z|yaw) (j,i) = integral (der(t^i , derive_order) * der(t^j ,derive_order) , ts , ts+1)
* Finally 
*					H = diag(Ho , H1 , ... , Hm) where m is number of segments
*/
void generate_cost_matrix_1d(double min_dur_segment,int N , int derive_order,const VectorXd &t, vector<T> &H_vect , MatrixXd M_diag[]){
	int M = t.size()-1;

	//We pre-compute the coefficients that will be multiply by (ts+1^pow_val - ts^powval)
	//In order to go faster we will only save the lower diagonal value since Matrix is symmetric
	double compute_coeff[(N*(N+1))/2];
	int coeff_iter = 0;
	for (int i = 0; i< N ; i++ ){
		for(int j = 0; j<=i ; j++){
			if(i < derive_order || j < derive_order){
				compute_coeff[coeff_iter] = 0;
				coeff_iter++;
				continue; 
			}
			int h = 1;
			for(int o =0;o<derive_order;o++){
				h *= (i-o)*(j-o);
			}
			int pow_val =  i + j - 2*derive_order +1;
			compute_coeff[coeff_iter] = (2.0 * ((double) h))/((double) pow_val);
			coeff_iter++;
		}
	}

	//derivatives for the start point
	MatrixXd start_der_vect(derive_order+1,N);
	generate_derivatives(0.0,start_der_vect); 				//we integrate between 0 and ts - ti for numerical stability for every segment
	DiagonalMatrix<double,Eigen::Dynamic> upper_left_inv(derive_order+1);

	for(int i = 0; i<= derive_order;i++){
		upper_left_inv.diagonal()[i] = 1.0/start_der_vect(i,i);
	}

	#pragma omp for schedule(static)
		for (int s = 0; s<M ;s++){
			MatrixXd H_matrix = MatrixXd::Zero(N,N);
			//We compute the local H non zero coefficient and only LOWER matrix
			coeff_iter = 0;
			double diff_t = t(s+1) - t(s); 						//much more numerically stable instead of (ts+1^pow_val - ts^powval) and doesn't change the problem
			for (int k=0; k<N; k++){
				for (int p=0; p<=k; p++){
					if( k>=derive_order && p>=derive_order){
						int pow_val =  k + p - 2*derive_order +1;
						H_matrix(k,p) = (pow(diff_t,pow_val))*compute_coeff[coeff_iter];
					}
					coeff_iter++;
				}
			}

			MatrixXd M_inv = MatrixXd::Zero(N,N);
			if(diff_t >= min_dur_segment){
				MatrixXd lower_left_inv(derive_order+1,derive_order+1);
				MatrixXd lower_right_inv(derive_order+1,derive_order+1);
				//Need the list of derivatives for the end of the segment
				MatrixXd end_der_vect(derive_order+1,N);
				generate_derivatives(diff_t,end_der_vect);

				//lower_right_inv = (end_der_vect.block(0,N/2,N/2,N/2)).inverse();
				lower_right_inv = (end_der_vect.block(0,N/2,N/2,N/2)).colPivHouseholderQr().inverse();				//Far More stable accuracy (And prove accuracy) than PartialLU decomposition
				lower_left_inv = -1.0 * (lower_right_inv * end_der_vect.block(0,0,N/2,N/2).triangularView<Eigen::Upper>() * upper_left_inv);
				M_inv.block(0,0,N/2,N/2) = upper_left_inv;
				M_inv.block(N/2,0,N/2,N/2) = lower_left_inv;
				M_inv.block(N/2,N/2,N/2,N/2) = lower_right_inv;

				//Make the t(M(-1)) H t(M)
				MatrixXd temp = H_matrix.selfadjointView<Lower>() * M_inv;
				temp =  M_inv.transpose() * temp;
				for(int i = 0 ; i < temp.rows() ; i++){
					for(int j = 0 ; j<=i ;  j++){
						#pragma omp critical
							H_vect.push_back(T(s*N + i,s*N + j, temp(i,j)));
					}
				}
			}else{
				M_inv.setIdentity();
				for(int i = 0 ; i < N ; i++){
					for(int j = 0 ; j<=i ;  j++){
						#pragma omp critical
							H_vect.push_back(T(s*N + i,s*N + j, H_matrix(i,j)));
					}
				}
			}
			//Save the local inverted M matrix for getting back the polynomial coefficient
			M_diag[s] = M_inv;
		}
}

//Only use when solving the problem with with the non iterative method (integration inequality into the Qr problem)
bool generate_corridors(int N , int nc, const MatrixXd &x,const VectorXd &t, const VectorXd &corridors,
	vector<T> &A, vector<bnd_interval> &b , const MatrixXd M_inv[]){

	int available_corridor = 0;
	int M = x.rows() - 1;

	#pragma omp for schedule(static)
		for (int m = 0; m < M; m++){
			if (corridors(m) <=0){
				continue;
			}
			#pragma omp critical
				available_corridor++;
			VectorXd next_point = x.row(m+1);
			VectorXd current_point = x.row(m);
			VectorXd norm_vect = (next_point - current_point).normalized();
			VectorXd b_val = current_point - ((current_point.dot(norm_vect))*norm_vect);

			VectorXd poly_vec = VectorXd::Zero(N);
			for (int j = 1 ; j<= nc; j++){
				MatrixXd A_m = MatrixXd::Zero(1,N * x.cols()); //Matrix for the local corridors constraints'
				generate_pow((((double) j) * (t(m+1) - t(m))/(1+nc)),poly_vec);// Respecting 0 tf-ti convention
				for (int axis =0; axis<x.cols();axis++){

					for(int a_iter = 0 ; a_iter <N ; a_iter++){
						double value = poly_vec[a_iter]*(1-norm_vect(axis)*norm_vect(axis));
						A_m(0 , axis * N + a_iter) = value;
					}

					for (int axis_sub = 0;axis_sub<x.cols(); axis_sub++){
						if(axis_sub != axis){
							for(int a_iter = 0 ; a_iter <N ; a_iter++){
								double value = -poly_vec[a_iter]*norm_vect(axis_sub)*norm_vect(axis);
								A_m(0 , axis_sub * N + a_iter) = value;
							}
						}
					}

					double sup_value = b_val(axis) + (corridors(m));
					double inf_value = b_val(axis) - (corridors(m));
					int curr_ind;
					#pragma omp critical
					{
						curr_ind = b.size();
						b.push_back(bnd_interval(T(b.size(),0,inf_value),T(b.size(),0,sup_value))); 
					}
					for(int iter_axis = 0 ; iter_axis < x.cols() ; iter_axis++){
						MatrixXd temp = A_m.block(0,iter_axis * N , 1 , N) * M_inv[m];
						for(int i =0 ; i<N ; i++ ){
							if(temp(0,i) != 0){
								#pragma omp critical	
									A.push_back(T(curr_ind, iter_axis*N*M +m*N +i ,temp(0,i)));
							}
						}
					}
				}

			}
		}
	return (available_corridor > 0);
}

/*
* This function globally store in the list A  the non zero element of A * inv(M)
* Where M is defined as in the function that generate the cost matrix
* Basically in the new problem formulation 
* x_new = M * x
* A_new = A * inv(M)
* b_new = b 
*/
bool generate_deriv_order_constraints(int N, int derive_order, const MatrixXd &x,const new_time_scale &time_scale,
				const vector<ConstraintValue> constraints[],
					vector<T> &A, vector<bnd_interval> &b , const MatrixXd M_inv[]){

	int M = x.rows() -1;
	bool contain_inequality = false;

	MatrixXd start_der_vect(derive_order+1,N);
	generate_derivatives(0.0,start_der_vect); // we integrate between 0 and ts - ti for numerical stability

	#pragma omp for schedule(static)
		for (int m=0;m<M;m++){

			//constraint for the segment start point
			MatrixXd A_m_init = start_der_vect * (M_inv[m]);

			//constraint for the segment end point
			MatrixXd end_der_vect(derive_order+1,N);
			generate_derivatives((time_scale.t(m+1)-time_scale.t(m)),end_der_vect);
			MatrixXd A_m_end = end_der_vect * M_inv[m];

			for (int axis = 0;axis<x.cols(); axis++){
				int curr_ind;
				#pragma omp critical
				{
					curr_ind = b.size();
					b.push_back(bnd_interval(T(b.size(),0,x(m,axis)),T(b.size(),0,x(m,axis))));
					b.push_back(bnd_interval(T(b.size(),0,x(m+1,axis)),T(b.size(),0,x(m+1,axis))));
				}
				for(int a_iter =0; a_iter< N ; a_iter++){
					if (A_m_init(0,a_iter) != 0){
						#pragma omp critical
							A.push_back(T(curr_ind,axis*N*M+m*N+ a_iter,A_m_init(0,a_iter)));
					}
				}
				for(int a_iter =0; a_iter< N ; a_iter++){
					if (A_m_end(0,a_iter) != 0){
						#pragma omp critical
							A.push_back(T(curr_ind+1,axis*N*M+m*N+ a_iter,A_m_end(0,a_iter)));
					}
				}
			}

			//User constraints (On the different derivatives of the current setpoint ) and initialisation to false for security
			bool already_constrains[derive_order];
			for (int d = 0;d<derive_order;d++){
				already_constrains[d] = false;
			}
			
			if(!constraints[m].empty()){
				for (vector<ConstraintValue>::const_iterator c = constraints[m].begin() ; c!= constraints[m].end();++c){
					if(!c->bndl_valid && !c->bndu_valid){
						continue;
					}
					double scale_pow = pow(time_scale.scale_factor,c->derive_order);
					double inf_value = c->bndl_valid ? c->bndl/scale_pow : -MAX_REAL_VALUE;
					double sup_value = c->bndu_valid ? c->bndu/scale_pow : MAX_REAL_VALUE;
					int curr_ind;
					#pragma omp critical
					{
						curr_ind = b.size();
						b.push_back(bnd_interval( T(b.size(),0, inf_value),T(b.size(),0,sup_value)));
					}

					for(int a_iter =0; a_iter< N ; a_iter++){
						if (A_m_init(c->derive_order,a_iter) != 0){
							#pragma omp critical
								A.push_back(T(curr_ind,c->label*N*M+m*N+ a_iter,A_m_init(c->derive_order,a_iter)));
						}
					}

					if( !(c->bndl_valid && c->bndu_valid && inf_value == sup_value)){
						#pragma omp critical
							contain_inequality = true;
					}
				}
			}
			if(! constraints[m+1].empty()){
				for (vector<ConstraintValue>::const_iterator c = constraints[m+1].begin() ; c!= constraints[m+1].end();++c){
					if(!c->bndl_valid && !c->bndu_valid){
						continue;
					}
					double scale_pow = pow(time_scale.scale_factor,c->derive_order);
					double inf_value = c->bndl_valid ? c->bndl/scale_pow : -MAX_REAL_VALUE;
					double sup_value = c->bndu_valid ? c->bndu/scale_pow : MAX_REAL_VALUE;
					int curr_ind;
					#pragma omp critical
					{
						curr_ind =  b.size();
						b.push_back(bnd_interval(T(b.size(),0, inf_value),T(b.size(),0,sup_value)));
					}

					for(int a_iter =0; a_iter< N ; a_iter++){
						if(A_m_end(c->derive_order,a_iter) != 0){
							#pragma omp critical
								A.push_back(T(curr_ind,c->label*N*M+m*N+ a_iter, A_m_end(c->derive_order,a_iter)));
						}
					}

					if( !(c->bndl_valid && c->bndu_valid && inf_value == sup_value)){
						#pragma omp critical
							contain_inequality = true;
					}
					already_constrains[c->derive_order-1] = true;
				}
			}
			//Handle non user constraints --> continuity  for all the p derivative for the intermediate points
			for(int i = 1; i<= derive_order; i++){
				if (already_constrains[i-1] || m>= M-1){
					continue;
				}
				for (int axis = 0 ; axis<x.cols() ; axis++){
					int curr_ind;
					#pragma omp critical
					{
						curr_ind = b.size();
						b.push_back(bnd_interval(T(b.size(),0, 0.0),T(b.size(),0,0.0)));
					}
					for(int a_iter =0; a_iter< N ; a_iter++){
						if( A_m_end(i,a_iter) != 0){
							#pragma omp critical
							{
								A.push_back(T(curr_ind,axis*N*M + m*N+ a_iter, A_m_end(i,a_iter)));
								A.push_back(T(curr_ind,axis*N*M + (m+1)*N + a_iter, -A_m_init(i,a_iter))); //Since next segment is between 0 and ts-ti
							}
						}
					}
				}
			}
		}
	return contain_inequality;
}

/*
*	Lagrange multiplier solver when the problem doesn't contains inequality
*	Use of SPQR which allows multithreading --> fast time Ax = b solve if openblas , lapack or GPU use have been installed according
*	To the documentation.
*	Solve the problem M.Y = X 
*	Where M = [H  t(A)]
*			  [A  0]
*	and X = [0]  with Y = [var_sy]
*		    [b]			  [lambda]
*	vector<T> M_vec;
*/
double lagrange_solve(bool return_cost, const solver_param &params ,const double scale_factor, const int M , const int dim_,const vector<T> &H , const vector<T> &A , const vector<bnd_interval> &b , VectorXd &sol){


	const int lin_const = b.size();
	const int N = (params.derive_order+1)*2;
	int dim_H = dim_*N*M;
	vector<T> m_sparse_list;
	SparseMatrix<double> H_matrix(dim_H,dim_H);

	for (vector<T>::const_iterator loc = H.begin() ; loc != H.end() ; loc++){
		for(int i=0 ; i<dim_ ; i++){
			m_sparse_list.push_back(T(i*N *M + loc->row(),i*N *M + loc->col(),loc->value()));
		}
	}

	//H_matrix will be used for cost calculation
	if(return_cost || params.verbose){
		H_matrix.setFromTriplets(m_sparse_list.begin() , m_sparse_list.end());
	}

	for (vector<T>::const_iterator a_sub = A.begin() ; a_sub != A.end() ; a_sub++){
		m_sparse_list.push_back(T(dim_H+a_sub->row(),a_sub->col(),a_sub->value()));
	}

	//Initialization and fill in of M matrix
	SparseMatrix<double> M_matrix(dim_H+lin_const,dim_H + lin_const);
	M_matrix.setFromTriplets(m_sparse_list.begin() , m_sparse_list.end());
	M_matrix.makeCompressed();

	//Initialisation of X (the right part of the system to solve)
	VectorXd X = VectorXd::Zero(dim_H + lin_const);
	for( vector<bnd_interval>::const_iterator b_sub = b.begin(); b_sub != b.end() ; b_sub++){
		X(dim_H+b_sub->first.row()) = b_sub->first.value();
	}

	//Use a QR solver since M_matrix is not always non singular
	Eigen::SPQR< Eigen::SparseMatrix<double> > solverA;
	solverA.compute(M_matrix.selfadjointView<Lower>());

 	//Eigen::VectorXd Y = solverA.solve(X);
 	sol = solverA.solve(X).head(dim_H);

 	if(return_cost || params.verbose){
 		VectorXd x_temp = H_matrix.selfadjointView<Lower>() * sol;
 		double cost = sol.transpose() * x_temp;
 		cost *= 0.5 * std::pow(scale_factor , 2*params.derive_order - 1);
 		if(params.verbose){
 			SparseMatrix<double> A_matrix(lin_const,dim_H);
			VectorXd b_matrix = VectorXd::Zero(lin_const);
			A_matrix.setFromTriplets(A.begin(),A.end());
			for( vector<bnd_interval>::const_iterator b_sub = b.begin(); b_sub != b.end() ; b_sub++){
				b_matrix(b_sub->first.row()) = b_sub->first.value();
			}
			cout << "(Ax-b).maxCoeff = " << (A_matrix*sol - b_matrix).maxCoeff() << endl;
 			cout << "(Ax-b).minCoeff = " << (A_matrix*sol - b_matrix).minCoeff() << endl;
 			cout << "(Ax-b).norm = " << (A_matrix*sol - b_matrix).norm() << endl;
 			cout << "Cost Lagrange = " << cost << endl;
 			cout << "------------------------------" << endl;
 		}
 		return cost;
	}
	return 0.0;
}

double barrier_min_f(const gsl_vector *x , void *params){

	//cout << "Function evaluation in "  << endl;
	struct barrier_params *f_params = (barrier_params *) params;
	double barrier_cost = 0;
	for(int i=0 ; i< x->size ; i++){
		f_params->X->operator()(f_params->H_matrix->cols() + f_params->arr_ind_ineq[i].first.row()) = gsl_vector_get(x , i);
		if(f_params->arr_ind_ineq[i].first.value() == -MAX_REAL_VALUE && f_params->arr_ind_ineq[i].second.value() != MAX_REAL_VALUE){
			if( f_params->arr_ind_ineq[i].second.value()- gsl_vector_get(x , i) <= 0){
				//return GSL_NAN;
				return MAX_REAL_VALUE;
			}
			barrier_cost += std::log(f_params->arr_ind_ineq[i].second.value()- gsl_vector_get(x , i));
		}else if(f_params->arr_ind_ineq[i].first.value() != -MAX_REAL_VALUE && f_params->arr_ind_ineq[i].second.value() == MAX_REAL_VALUE){
			if( gsl_vector_get(x , i)-f_params->arr_ind_ineq[i].first.value() <= 0){
				return MAX_REAL_VALUE;
			}
			barrier_cost += std::log(gsl_vector_get(x , i)-f_params->arr_ind_ineq[i].first.value());
		}else if(f_params->arr_ind_ineq[i].first.value() != -MAX_REAL_VALUE && f_params->arr_ind_ineq[i].second.value() != MAX_REAL_VALUE){
			if( f_params->arr_ind_ineq[i].second.value()- gsl_vector_get(x , i) <= 0 || (gsl_vector_get(x , i)-f_params->arr_ind_ineq[i].first.value()) <= 0){
				return MAX_REAL_VALUE;
			}
			barrier_cost += std::log(f_params->arr_ind_ineq[i].second.value()- gsl_vector_get(x , i)) + std::log(gsl_vector_get(x , i)-f_params->arr_ind_ineq[i].first.value());
		}
	}
	VectorXd sol = f_params->solver->solve(*(f_params->X)).head(f_params->H_matrix->cols());
	VectorXd temp = (f_params->H_matrix->selfadjointView<Lower>() * sol);
	double res = sol.transpose() * temp;
	f_params->barrier_cost = f_params->mu0 * barrier_cost;
	//cout << "Function evaluation = " << res/2 - f_params->barrier_cost << endl;
	/*for(int i =0 ; i< x->size ; i++){
		cout << gsl_vector_get(x , i)  << "  ";
	}*/
	//cout << endl;
	return (((res/2)*f_params->scale_factor) - f_params->barrier_cost);
}

void barrier_min_df(const gsl_vector *x , void *params , gsl_vector *df){

	struct barrier_params *f_params = (barrier_params *) params;

	VectorXd sol(f_params->H_matrix->cols()) , temp(f_params->H_matrix->cols());
	for(int i = 0 ; i< x->size ; i++ ){
		f_params->X->operator()(f_params->H_matrix->cols() + f_params->arr_ind_ineq[i].first.row()) = gsl_vector_get(x , i) + f_params->h;
		sol = f_params->solver->solve(*(f_params->X)).head(f_params->H_matrix->cols());
		temp = (f_params->H_matrix->selfadjointView<Lower>() * sol);
		double res_h_sup = sol.transpose() * temp;
		res_h_sup *= 0.5 * f_params->scale_factor;

		f_params->X->operator()(f_params->H_matrix->cols() + f_params->arr_ind_ineq[i].first.row()) = gsl_vector_get(x , i) - f_params->h;
		sol = f_params->solver->solve(*(f_params->X)).head(f_params->H_matrix->cols());
		temp = (f_params->H_matrix->selfadjointView<Lower>() * sol);
		double res_h_inf = sol.transpose() * temp;
		res_h_inf *= 0.5 * f_params->scale_factor;

		double cost_barrier_i = 0;
		//Gradient of the verhead log function
		if(f_params->arr_ind_ineq[i].first.value() == -MAX_REAL_VALUE && f_params->arr_ind_ineq[i].second.value() != MAX_REAL_VALUE){
			cost_barrier_i = -1.0/(f_params->arr_ind_ineq[i].second.value()- gsl_vector_get(x , i));
		}else if(f_params->arr_ind_ineq[i].first.value() != -MAX_REAL_VALUE && f_params->arr_ind_ineq[i].second.value() == MAX_REAL_VALUE){
			cost_barrier_i = 1.0/(gsl_vector_get(x , i)-f_params->arr_ind_ineq[i].first.value());
		}else if (f_params->arr_ind_ineq[i].first.value() != -MAX_REAL_VALUE && f_params->arr_ind_ineq[i].second.value() != MAX_REAL_VALUE){
			cost_barrier_i = (-1.0/(f_params->arr_ind_ineq[i].second.value()- gsl_vector_get(x , i))) + (1.0/(gsl_vector_get(x , i)-f_params->arr_ind_ineq[i].first.value()));
		}
		//cout << ((res - grad_qr)/f_params->h) - (f_params->mu0 * cost_barrier_i) << "  ";
		gsl_vector_set(df,i, ((res_h_sup - res_h_inf )/(2*f_params->h)) - (f_params->mu0 * cost_barrier_i));
		f_params->X->operator()(f_params->H_matrix->cols() + f_params->arr_ind_ineq[i].first.row()) = gsl_vector_get(x , i);
	}
	//cout << endl;
}

void barrier_min_fdf(const gsl_vector *x , void *params , double *f ,  gsl_vector *df){
	struct barrier_params *f_params = (barrier_params *) params;
	*f = barrier_min_f(x , params);
	barrier_min_df(x , params , df);
}

/*
* Version of minimum snap supporting inequality constraints on the differents derivatives order
* We principally use the logarith barrier method for computing the minimisation problem on
* The inequality constraints variabbles
* Summary: Combinaison of line search | gradient descent method with barrier method 
*		   Instead of Newton iteration algorithm (Too much overhead with computating the hessien)
*	Interval constraints a <= x <= b ------> (x-a) >=0
*											 (b-x) >=0
*	This is done for every boundary constraints. If there exist only one bound , we ignore the second one in the barrier formulation
*	Typically for non infinite bound a , b  feasible set is a+b/2 (But not sure it is the optimal one)
*/

double compute_min_snap_with_inequality_1(bool return_cost, const solver_param &params, const double scale_factor, const int M, 
											const int dim_, const std::vector<T> &H , const vector<T> &A, 
													const vector<bnd_interval> &b ,VectorXd &sol){
	const int lin_const = b.size();
	const double h_step = 1e-6;
	const double init_step_size = 1e-3;		//0.001
	const double tol = 1e-3;					//1e-3
	const double grad_rel_error = 1e-3;			//1e-3
	const int max_inner_loop = 100;				//log(1/epsilon)
	const double const_feasible_value = 1.0;	//TODO change it to a more compatible value for the other inequality value reference --> Still This one is good for setting to zero the log function

	//Finding the inequality constraints
	int nb_ineq_var = 0;

	//First iteration to count the number of inequality constraints
	for(vector<bnd_interval>::const_iterator linear_const = b.begin(); linear_const != b.end(); linear_const++){
		if(linear_const->first.value() != linear_const->second.value()){
			nb_ineq_var++;
		}
	}

	bnd_interval arr_ind_ineq[nb_ineq_var];
	VectorXd ineq_init(nb_ineq_var);

	//Second iteration save it indices and set the initial feasible point for the logarithm barrier method
	int iter = 0;
	for(vector<bnd_interval>::const_iterator linear_const = b.begin(); linear_const != b.end(); linear_const++){
		if(linear_const->first.value() != linear_const->second.value()){
			//cout << "bndl = " << linear_const->first.value() << "bndu = " << linear_const->second.value() << endl;
			//saves the indices for later easy modification of the b matrix in the lagrange problem
			arr_ind_ineq[iter] = *linear_const;
			//We choose a feasible init point
			if(linear_const->first.value() == -MAX_REAL_VALUE ){
				ineq_init(iter) = linear_const->second.value() == MAX_REAL_VALUE ? 0.0 :(linear_const->second.value()-const_feasible_value);
			}else if(linear_const->second.value() == MAX_REAL_VALUE){
				ineq_init(iter) = linear_const->first.value() + const_feasible_value;
			}else{
				ineq_init(iter) = (linear_const->first.value() + linear_const->second.value())/2.0;
			}
			iter++;
		}
	}

	cout << ineq_init.transpose() << endl;

	//We solve the problem with the initial feasible point
	const int N = (params.derive_order+1)*2;
	int dim_H = dim_*N*M;
	vector<T> m_sparse_list;
	SparseMatrix<double> H_matrix(dim_H,dim_H);

	for (vector<T>::const_iterator loc = H.begin() ; loc != H.end() ; loc++){
		for(int i=0 ; i<dim_ ; i++){
			m_sparse_list.push_back(T(i*N *M + loc->row(),i*N *M + loc->col(),loc->value()));
		}
	}

	//H_matrix will be used for cost calculation
	H_matrix.setFromTriplets(m_sparse_list.begin() , m_sparse_list.end());

	//Linear constraints Matrix
	for (vector<T>::const_iterator a_sub = A.begin() ; a_sub != A.end() ; a_sub++){
		m_sparse_list.push_back(T(dim_H+a_sub->row(),a_sub->col(),a_sub->value()));
	}

	//Initialization and fill in of M matrix
	SparseMatrix<double> M_matrix(dim_H+lin_const,dim_H + lin_const);
	M_matrix.setFromTriplets(m_sparse_list.begin() , m_sparse_list.end());
	M_matrix.makeCompressed();

	//Initialisation of X (the right part of the system to solve)
	VectorXd X = VectorXd::Zero(dim_H + lin_const);
	int iter_b = 0;
	for( vector<bnd_interval>::const_iterator b_sub = b.begin(); b_sub != b.end() ; b_sub++){
		if(b_sub->first.value() == b_sub->second.value()){
			X(dim_H+b_sub->first.row()) = b_sub->first.value();
		} else{
			X(dim_H+b_sub->first.row()) = ineq_init(iter_b);
			iter_b++;
		}
	}

	//Use a QR solver since M_matrix is not always non singular --> The decomposition will be store once for all the gradient calculation
	//Obviously since we already have the QR decomposition strategie allow us to go faster on gradient calculation
	Eigen::SPQR< Eigen::SparseMatrix<double> > solverA;
	solverA.compute(M_matrix.selfadjointView<Lower>());

 	//Logarithm barrier method constant
	double mu0 = 0.001;				//0.1
	const double mu_factor = 0.01;	//0.01
	const double desired_accuracy = 1e-20;
	const int max_iteration = ((int) (std::log(nb_ineq_var*(mu0/desired_accuracy))/ std::log(1.0/mu_factor))) + 1;	//log(m*mu/eps)/log(1/mu_factor) + 1

	cout << "Total Iteration = " << max_iteration << endl;

	//Transforming initial set in gsl vector 
	gsl_vector *x;
	x = gsl_vector_alloc(nb_ineq_var);
	for(int i = 0; i<nb_ineq_var ; i++){
		gsl_vector_set(x , i , ineq_init(i));
	}

	//Set the defaul parameters of the functions to minimize
	struct barrier_params params_min_solver;
	params_min_solver.solver = &solverA;
	params_min_solver.H_matrix = &H_matrix;
	params_min_solver.X = &X;
	params_min_solver.arr_ind_ineq = arr_ind_ineq;
	params_min_solver.h = h_step;
	params_min_solver.mu0 = mu0;
	params_min_solver.scale_factor = std::pow(scale_factor, 2* params.derive_order - 1);

	//Initialize the functions to minimized by gnu
	gsl_multimin_function_fdf func_to_min;
	func_to_min.n = nb_ineq_var;
	func_to_min.f = &barrier_min_f;
	func_to_min.df = &barrier_min_df;
	func_to_min.fdf = &barrier_min_fdf;

	func_to_min.params = (void *) &params_min_solver;

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_vector_bfgs2;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T , nb_ineq_var);
	gsl_multimin_fdfminimizer_set(s , &func_to_min , x , init_step_size, tol);

	int outer_loop = 0;
	int inner_loop =0;
	int status;
	while(outer_loop < max_iteration){
		do {
			inner_loop++;
			status = gsl_multimin_fdfminimizer_iterate(s);
			if(status){
				break;
			}
			status = gsl_multimin_test_gradient(s->gradient , grad_rel_error);
		}while(status == GSL_CONTINUE && inner_loop < max_inner_loop);
		gsl_vector *cur_min = gsl_multimin_fdfminimizer_x(s);
		double val_min = gsl_multimin_fdfminimizer_minimum(s);
		gsl_multimin_fdfminimizer_restart(s);
		cout << "Inner loop = " << inner_loop << endl;
		cout << "Val min = " << val_min << endl;
		inner_loop =0;
		mu0 *= mu_factor;
		params_min_solver.mu0 = mu0;
		outer_loop++;
	}
	gsl_vector *cur_min = gsl_multimin_fdfminimizer_x(s);
	for(int i=0;i<nb_ineq_var ; i++){
		X(dim_H+arr_ind_ineq[i].first.row()) = gsl_vector_get(cur_min,i);
	}
	sol = solverA.solve(X).head(dim_H);

	double cost =0;

	if(return_cost){
		VectorXd x_temp = H_matrix.selfadjointView<Lower>() * sol;
	 	cost = sol.transpose() * x_temp;
	 	cost =  cost/2;
	 	if(params.verbose){
	 		cout << "Final cost  = " << cost << endl;
	 	}
	}
	
	//We free the vector we allocate previously
	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return cost;
}


/*
* Compute the default min_snap with corridors and inequality/equality constraints based on the choice of the solver
* This function save the cost in cost params and the time scaled in struc new_time_scale
* ret_cost_init_var = true if user want to return the cost value but NOT the initial variable problem
* 					  false if user want to return the solution in the initial frame system but not the cost
*/
bool compute_min_snap_default_global(bool ret_cost_init_var, const solver_param &params, const MatrixXd &x,new_time_scale &time_scale,
	const vector<ConstraintValue> constraints[] ,const VectorXd &corridors,VectorXd &sol , double &cost){

	const int N = (params.derive_order+1)*2;

	//Check if the dimensions of the problem are OK
	if(x.rows() != time_scale.t.size()){
		cout << "waypoints, time have not the same size !!!!" <<endl;
		cost = MAX_REAL_VALUE;
		return false;
	}
	if(corridors.size() != x.rows()-1 ){
		cout << "inconsistent value M and corridors list size constraints ! Fill it with 0 if no use ..." << endl;
		cost = MAX_REAL_VALUE;
		return false;
	}

	//First thing is to rescale the time for easy matrix inversion M --> the initial scale factor has always to be 1 at the begining
	time_scale.t_init = time_scale.t(0);
	time_scale.t(0) = 0.0;
	double min_duration = time_scale.t(1) -  time_scale.t_init;
	for(int i = 1; i< time_scale.t.size() ; i++){
		time_scale.t(i) =  time_scale.t(i) - time_scale.t_init;
		if(min_duration > (time_scale.t(i) - time_scale.t(i-1))){
			min_duration =  (time_scale.t(i) - time_scale.t(i-1));
		}
	}
	time_scale.t *= params.min_scale_duration / min_duration ;
	time_scale.scale_factor *= params.min_scale_duration / min_duration;

	//Number of segment and Number of variables
	int M = x.rows() - 1;
	int dim_ = x.cols();

	//Initialize solution vector
	sol = VectorXd(N * M * dim_);

	//Use sparse properties of cost Matrix and A to fasten the solve of the problem
	//sparse Cost vector for each axis is the same , sparce linear constraints vector , comparison vector
	vector<T> H_m , A; 
	vector<bnd_interval> b ;
	MatrixXd M_inv[M];

	//pre allocation of H_m size = max size that can possibly take H_m (base on his symmetric spec)
	H_m.reserve(((N*(N+1))/2)*M);

	//pre alloc of A , b and rel_a_b --> simple estimation
	A.reserve(2*(M+1)*dim_ + M*params.derive_order);
	b.reserve(2*(M+1)*dim_ + M*params.derive_order);

	//generate 1 axis = N*M , N*M  cost Matrix
	generate_cost_matrix_1d(params.min_scale_duration,N,params.derive_order,time_scale.t,H_m,M_inv);

	//generate linear constraints except from corridors constraints
	bool has_ineq = generate_deriv_order_constraints(N,params.derive_order,x,time_scale,constraints,A,b,M_inv);

	//Use the same instance for the generation of corridors cosntraints
	//if( params.use_alglib_corr && x.cols() > 1){ //No need to generate corridors since it doesn't make sense in 1D problem
	//	has_ineq = has_ineq || generate_corridors(N,params.nc,x,time_scale.t,corridors,A,b,M_inv);
	//}

	//Depending on if the system contains inequality or not it may be useful to use lagrange multiplier to go faster
	if(has_ineq){
		//cout << " Logarithm barrier solver choosed ...." << endl;
		cost = compute_min_snap_with_inequality_1(ret_cost_init_var,params,time_scale.scale_factor,M,dim_,H_m,A,b,sol);
	}else{
		//cout << "Lagrange multiplier was automatically choosed ..." << endl;
		cost = lagrange_solve(ret_cost_init_var,params,time_scale.scale_factor,M,dim_,H_m,A,b,sol);
	}

	if(! ret_cost_init_var){
		for (int i =0; i< x.cols() ; i++ ){
			for (int j = 0 ; j<M ; j++){
				sol.segment(i * N * M + j*N,N) = M_inv[j] * sol.segment(i * N * M + j*N,N) ; //M_inv * x_new
			}
		}
	}
	return true;
}

bool compute_min_snap_default(const solver_param &params, const MatrixXd &x,new_time_scale &time_scale,
	const vector<ConstraintValue> constraints[],const VectorXd &corridors,VectorXd &sol){
	double cost = 0;
	return compute_min_snap_default_global(false , params , x , time_scale , constraints , corridors , sol ,cost);
}


/*
* Gradient descent implementation for finding optimal time segment with Minimum snap
* The tuning parameters are given in the params of the launch file
* The results cost is not guarantee to be a global minimum
*/
bool gradient_descent_minsnap_optimal(const solver_param &params, const MatrixXd &x,double t_init_,double t_end_,
					const vector<ConstraintValue> constraints[],
						const VectorXd &corridors, VectorXd &sol_x, new_time_scale &time_scale){
	double cost;

	//Compute initial guess based on the current distance between point
	VectorXd t(x.rows()-1);
	time_scale.t = VectorXd(x.rows());

	//initial estimation
	time_scale.t_init = t_init_;
	time_scale.scale_factor = 1.0;
	t = VectorXd::Constant(x.rows()-1,(t_end_-t_init_)/(x.rows()-1));
	time_scale.t(0) = t_init_;
	for(int i =1 ; i< x.rows() ; i++){
		time_scale.t(i) =  t_init_ + i * t(i-1);
	}

	//Compute initial cost and check if there is a valid solution
	compute_min_snap_default_global(true,params,x,time_scale,constraints,corridors,sol_x,cost);
	t *= time_scale.scale_factor;
	//cout << "-------------------- " << time_scale.t(x.rows()-1) << " ---- " << time_scale.scale_factor << " ------------------" << endl;
	//double t_init = time_scale.t(0);
	//double t_end = time_scale.t(x.rows() - 1);


	if( cost == MAX_REAL_VALUE){
		cout << "First estimation with distance between point is MAX_VALUE ---> FAILED " << endl;
		return false;
	}

	int M = x.rows() -1 ;

	const double coeff_val = -1.0 / (M-1);
	const double h_step = 1e-6;
	//const double c1 = 0.5;
	//const double c2 = 0.5;

	//Gradient vector
	VectorXd gradient(M);
	//VectorXd solution(sol_x.size());

	//relative error to reach
	double step = MAX_REAL_VALUE;
	int iteration_number = 0;
	int call_minsnap = 0;

	while(step > params.epsilon){
		//Save min value of duration and max Value of gradient for scaling it for the problem
		double min_t = MAX_REAL_VALUE;
		double max_gradient= -MAX_REAL_VALUE;

		//directive vector
		VectorXd g = VectorXd::Constant(M,coeff_val);
		for (int i = 0 ; i< M ; i++){
			g(i) = 1.0;
			new_time_scale time_scale_temp;
			time_scale_temp.t = VectorXd(M+1);
			time_scale_temp.scale_factor = time_scale.scale_factor;
			time_scale_temp.t(0) = t_init_;
			for(int j = 1 ; j <= M ; j++){
				time_scale_temp.t(j) = time_scale_temp.t(j-1) + t(j-1) + (h_step * g(j-1)); 
			}
			g(i) = coeff_val;
			double cost_i;
			compute_min_snap_default_global(true,params,x,time_scale_temp,constraints,corridors,sol_x,cost_i);
			call_minsnap++;
			gradient(i) = (cost_i - cost)/h_step;
			max_gradient = abs(gradient(i))>max_gradient ? abs(gradient(i)) : max_gradient ;
			min_t = abs(t(i)) <min_t ? abs(t(i)) : min_t;
		}
		//cout << gradient.transpose() << endl;
		//Normalize gradient vector since it can have huge values or bad constraints for time
		//cout << "Gradient = " << gradient << endl;

		gradient *= (min_t/max_gradient);

		//cout << gradient.transpose() << endl;
		//Backtracking line search algorithm
		double alpha = params.init_alpha ;
		double curr_cost = MAX_REAL_VALUE;
		while(curr_cost > cost){
			if (alpha <= params.minAlpha){
				break;
			}
			VectorXd T_alpha = t - (alpha * gradient);
			//const double prop_coeff = (t_end_ - t_init_)/T_alpha.sum();
			const double prop_coeff = (time_scale.t(M))/T_alpha.sum();
			T_alpha *= prop_coeff;
			//VectorXd curr_t(M+1);
			new_time_scale time_scale_temp;
			time_scale_temp.t = VectorXd(M+1);
			time_scale_temp.t(0) = t_init_;
			time_scale_temp.scale_factor = time_scale.scale_factor;
			for(int i=1 ; i<= M ; i++){
				time_scale_temp.t(i) = time_scale_temp.t(i-1) + T_alpha(i-1);
			}
			compute_min_snap_default_global(true,params,x,time_scale_temp,constraints,corridors,sol_x,curr_cost);
			call_minsnap++;
			if(curr_cost < cost){
				t= T_alpha;
				time_scale.t(0) = t_init_;
				for(int i=1 ; i<= M ; i++){
					time_scale.t(i) = time_scale.t(i-1) + T_alpha(i-1);
				}
				//cout << "t_end = " << time_scale.t(M) << "   scale_factor = "<< time_scale.scale_factor << endl;
				break;
			}
			alpha = alpha * 0.5 ;
		}

		/*double curr_cost = MAX_REAL_VALUE;
		double alpha = params.init_alpha;
		double suff_decrease = c1 * gradient.norm();
		gradient.normalize();
		while(alpha > params.minAlpha){
			//if (alpha <= params.minAlpha){
			//	break;
			//}
			VectorXd T_alpha = t - (alpha * gradient);
			const double prop_coeff = (t_end_ - t_init_)/T_alpha.sum();
			T_alpha *= prop_coeff;
			//VectorXd curr_t(M+1);
			new_time_scale time_scale_temp;
			time_scale_temp.t = VectorXd(M+1);
			time_scale_temp.t(0) = t_init_;
			time_scale_temp.scale_factor = time_scale.scale_factor;
			int i;
			for(i=1 ; i<= M ; i++){
				if(T_alpha(i) <= 0){
					i = M +2;
					continue;
				}
				time_scale_temp.t(i) = time_scale_temp.t(i-1) + T_alpha(i-1);
			}
			if(i != (M+1)){
				alpha *= c2;
			}else{
				compute_min_snap_default_global(true,params,x,time_scale_temp,constraints,corridors,sol_x,curr_cost);
				call_minsnap++;
				if(cost - curr_cost >= alpha * suff_decrease){
					t= T_alpha;
					time_scale.t(0) = t_init_;
					for(int i=1 ; i<= M ; i++){
						time_scale.t(i) = time_scale.t(i-1) + T_alpha(i-1);
					}
					break;
				}else{
					alpha = alpha * c1 ;
				}
			}
		}*/

		if(alpha < params.minAlpha){
			break;
		}
		step = (cost - curr_cost)/cost;
		cost = curr_cost;
		iteration_number++;
	}

	if(params.verbose){
		cout << "-----------------------------------------------------------------" << endl;
		cout <<"Number of call to default minsnap = " << call_minsnap << endl;
		cout << "Number of iteration = " << iteration_number << endl;
		cout << "-----------------------------------------------------------------" << endl;
	}
	return compute_min_snap_default(params, x , time_scale , constraints , corridors ,sol_x);
}

bool compute_min_snap_iter_corridors(bool optima_segment, const solver_param &params,MatrixXd &x,new_time_scale &time_scale,
	const vector<ConstraintValue> constraints[] ,VectorXd &corridors, VectorXd &sol){

	if(params.verbose){
		cout << "Minimum snap with iterative corridors chosen ..." << endl;
	}
	//bool satisfy_constrain = false;
	//int iter_points = 4;
	int M = x.rows() - 1;
	const int N = (params.derive_order +1) * 2;

	vector<int> ind_corr;			//vector storing indices of segment where the trajectory doesn't respect safe corridors
	vector<VectorXd> pos_add;		//vector saving the intermediate positions to add to these segment before recomputing mins_snap
	vector<double> time_add;		//vector saving to time corresponding to these new added point
	vector<ConstraintValue> no_constraint;	//Just a no constraints vector for new added points

	ind_corr.reserve(M);
	pos_add.reserve(M);
	time_add.reserve(M);

	//relative distance from corridors where unbound point can be projected
	const double rel_err = 0.05; // 5%

	VectorXd temp(1);				//Pre allocate scalar for some matricial products
	int iter_number = 0;			//Number of iteration
	int added_waypoints =0;			//Number of supplementary waypoints added due to corridors constraints
	int chunk_size = (M/omp_get_num_threads())/omp_get_num_threads();
	chunk_size = chunk_size == 0 ? 1 : chunk_size;

	if(optima_segment){
		cout << "Optimal time segment ..." << endl;
		if (! compute_min_snap_optimal_segment_times(params,x,time_scale.t(0),time_scale.t(M), constraints ,corridors, sol , time_scale)){
			return false;
		}
	}else{
		cout << "Default min_snap ..." << endl;
		if (! compute_min_snap_default(params,x,time_scale,constraints ,corridors, sol)){
			return false;
		}
	}

	while (true){
		iter_number++;
		int M = x.rows() - 1;

		#pragma omp for ordered schedule(dynamic , chunk_size)
			for (int i = 0 ; i <M ; i++ ){
				if(corridors(i)<= 0.0){
					continue;
				}
				//Line equation is pos_init + lamda * dir_vect
				double t_init = time_scale.t(i);
				double t_end = time_scale.t(i+1);
				VectorXd pos_init(x.cols());		 		//Initial point on this segment
				VectorXd dir_vect(x.cols());				//Director vector of the sgement
				MatrixXd deriv_init(1,N), deriv_end(1,N); 		//Just need position order 
				generate_derivatives(0.0,deriv_init);
				generate_derivatives(t_end-t_init,deriv_end);
				for ( int k = 0 ; k < x.cols() ;  k++){
					temp = deriv_init * (sol.segment(k*M*N + i*N,N));
					pos_init(k) =  temp(0);
					temp = deriv_end * (sol.segment(k*M*N + i*N,N));
					dir_vect(k) = temp(0) - pos_init(k);
				}
				const double norm_dir_vect = dir_vect.squaredNorm();
				//cout << "Pos init = " << pos_init.transpose() << endl;
				//cout << "dir vect =  " << dir_vect.transpose() << endl;

				MatrixXd current_point(x.cols() , N);
				for(int k = 0 ; k< x.cols() ; k++){
					current_point.block(k,0,1,N) = sol.segment(k*M*N + i*N,N).transpose();
				}
				MatrixXd current_point_der(x.cols() , N);
				for(int k = 0 ; k< x.cols(); k++){
					current_point_der(k,N-1) = 0;
					for(int j = 0 ; j<N-1 ; j++){
						current_point_der(k,j) = (j+1) * current_point(k,j+1);
					}
				}
				//cout << "derivate vect =  " << current_point_der << endl;
				//Modify polynomial current_point to be current_point - Pos_init
				current_point.block(0,0,x.cols(),1) -= pos_init;
				//Compute (p'.dot(u)/norm(u))
				VectorXd dot_inter = VectorXd::Zero(N);
				for(int k = 0; k< x.cols() ; k++){
					dot_inter += (dir_vect(k)/norm_dir_vect) * current_point_der.block(k,0,1,N).transpose();
				}
				for(int k= 0; k< x.cols() ; k++){
					current_point_der.block(k,0,1,N) -= (dot_inter.transpose() * dir_vect(k));
				}
				//Compyte p' - (p'.dot(u)/norm(u))u

				//p' . dot(p- p_init)
				MatrixXd final_der_mat = MatrixXd::Zero(x.cols() , 2*N);
				for(int k=0 ; k < x.cols() ; k++){
					for(int j = 0 ; j< N ; j++){
						final_der_mat.block(k,j,1,N) += current_point_der(k,j) * current_point.block(k,0,1,N);
					}
				}
				for(int k=1; k< x.cols() ; k++){
					final_der_mat.block(0,0,1,N) += final_der_mat.block(k,0,1,N);
				} 
				int poly_order;
				for(int k = 2*N -1 ; k>=0 ; k--){
					if(final_der_mat(0,k) != 0){
						poly_order = k;
						k = -1;
					}
				}
				double derive_poly[poly_order+1];
				for(int k = 0 ; k<= poly_order ; k++){
					derive_poly[k] = final_der_mat(0,k);
				}

				double res[poly_order * 2];
				gsl_poly_complex_workspace *w = gsl_poly_complex_workspace_alloc(poly_order+1);
				gsl_poly_complex_solve(derive_poly,poly_order+1,w,res);

				double t_max = 0;
				double distance = 0;
				double lambda =0;
				VectorXd pos_max(x.cols());	
				MatrixXd max_mat(1,N);
				for ( int k = 0 ; k < x.cols() ;  k++){
					temp = max_mat * (sol.segment(k*M*N + i*N,N));
					pos_max(k) =  temp(0);
				}
				for(int k =0 ; k< poly_order ; k++){
					if(res[2*k] > 0 && res[2*k] < (t_end-t_init) && res[2*k + 1] == 0){
						generate_derivatives(res[2*k] , max_mat);
						VectorXd pos_max_temp(x.cols());
						for ( int p = 0 ; p < x.cols() ;  p++){
							temp = max_mat * (sol.segment(p*M*N + i*N,N));
							pos_max_temp(p) =  temp(0);
						}
						VectorXd diff_pos = pos_max_temp - pos_init;
						double lambda_temp = diff_pos.dot(dir_vect)/norm_dir_vect;
						double distance_temp = (diff_pos - (lambda_temp * dir_vect)).norm();
						if(distance_temp > distance){
							distance = distance_temp;
							pos_max = pos_max_temp;
							t_max = res[2*k] + t_init;
							lambda = lambda_temp;
						}
					}
				}

				gsl_poly_complex_workspace_free(w);

				if( distance >= corridors(i)){

					VectorXd pos_H = pos_init +  lambda * dir_vect; 	//Projection H of the maximum point on the current segment
					VectorXd H_pos_max = pos_max - pos_H;				//Vector H - pos_max 
					H_pos_max.normalize();								//Normalization of that vector
					pos_max = pos_H + (H_pos_max * ((1.0 - rel_err) * corridors(i)));		//We want pos_max to be below the safe corridors at a relative distance from that corridors = rel_err 
					//cout << " proj max = " << pos_max << endl;
					#pragma omp critical
					{
						ind_corr.push_back(i);
						pos_add.push_back(pos_max);
						time_add.push_back(t_max);
					}
				}
			}
		if (ind_corr.size() == 0){
			break;
		}
		//Temporary Matrix to save the new waypoints because of the added points
		MatrixXd temp_x(x.rows()+ind_corr.size(),x.cols());
		VectorXd temp_t(time_scale.t.size()+ind_corr.size());
		VectorXd temp_corr(time_scale.t.size()+ind_corr.size()-1);
		vector<ConstraintValue> temp_const[time_scale.t.size()+ind_corr.size()];

		//vector<vector<ConstraintValue> >::iterator constraints_itr = constraints.begin();
		vector<VectorXd>::iterator pos_itr = pos_add.begin();
		vector<double>::iterator time_itr = time_add.begin();
		int prev_row = 0;
		int prev_row_old = 0;
		for(vector<int>::iterator ind_ptr= ind_corr.begin(); ind_ptr != ind_corr.end() ; ind_ptr++){

			int block_size = (*ind_ptr)+1-prev_row;				//block to copy from the previous waypoints
			temp_x.block(prev_row_old,0,block_size,x.cols()) = x.block(prev_row,0,block_size,x.cols());
			temp_t.segment(prev_row_old,block_size) =  time_scale.t.segment(prev_row,block_size);
			temp_corr.segment(prev_row_old,block_size) = corridors.segment(prev_row,block_size);

			int next_row = prev_row_old+block_size;
			for(int p = 0 ; p< x.cols();p++){
				temp_x(next_row,p) = (*pos_itr)(p);
			}
			//temp_x.block(next_row,0,1,x.cols()) = pos_itr->head(x.cols());
			temp_t(next_row) = *time_itr;
			temp_corr(next_row) = corridors(*ind_ptr);

			for(int b = 0; b< block_size ; b++ ){
				temp_const[prev_row_old+b] = constraints[prev_row+b];
			}
			temp_const[next_row] = no_constraint;

			pos_itr++;
			time_itr++;
			prev_row = (*ind_ptr)+1;
			prev_row_old = next_row+1; 
		}

		int block_size = x.rows() - prev_row;
		temp_x.block(prev_row_old,0,block_size,x.cols()) = x.block(prev_row,0,block_size,x.cols());
		temp_t.segment(prev_row_old,block_size) =  time_scale.t.segment(prev_row,block_size);
		temp_corr.segment(prev_row_old,block_size-1) = corridors.segment(prev_row,block_size-1);
		for(int b=0; b < block_size ; b++){
			temp_const[prev_row_old+b] = constraints[prev_row+b];
		}

		//First copy new time list
		time_scale.t = temp_t;

		if(optima_segment){
			if (! compute_min_snap_optimal_segment_times(params,temp_x,time_scale.t(0),time_scale.t(M), temp_const ,temp_corr, sol , time_scale)){
				return false;
			}
		}else{
			if (! compute_min_snap_default(params,temp_x,time_scale,temp_const ,temp_corr, sol)){
				return false;
			}
		}

		added_waypoints+= temp_x.rows() - x.rows();
		x = temp_x;
		corridors = temp_corr;
		constraints = temp_const;

		//cout << x << endl;
		//cout time_scale.t 
		ind_corr.clear();
		pos_add.clear();
		time_add.clear();
	}

	if(params.verbose){
		cout << "Iterative corridor::number of iteration = " << iter_number << endl;
		cout << "Total point added = " << added_waypoints << endl;
	}
	return true;
}

bool compute_min_snap_optimal_segment_times(const solver_param &params, const MatrixXd &x,double t_init,double t_end,
					const vector<ConstraintValue> constraints[],
						const VectorXd &corridors, VectorXd &sol_x, new_time_scale &time_scale){
	return gradient_descent_minsnap_optimal(params,x,t_init,t_end,constraints,corridors,sol_x,time_scale);
}