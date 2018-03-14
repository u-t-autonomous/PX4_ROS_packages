
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

#include <stdio.h>
#include "min_snap.h"
#include <ctime>
#include <algorithm>
#include <math.h>

#include <fstream>

using namespace std;
using namespace Eigen;

#define PI_2 3.14159265*2


void get_circle_trajectory(int freq , double duration , double R , VectorXd &x , VectorXd &y , VectorXd &t);
void get_fermat_spiral(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &t);
void get_lituus_spiral(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &t);
void get_conical_helix(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &z ,VectorXd &t);
void get_uniform_helix(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &z ,VectorXd &t);

int main(int argc, char* argv[]){

	//std::cout << std::setprecision(20) << std::fixed;
	/*************************** Set params for your problem *********************************/
	min_snap::solver_param params = DEFAULT_PARAMS;
	params.verbose = true;
	//params.epsilon = 1e-3;
	//params.minAlpha =  1e-3;
	//params.use_alglib_opt = false;
	/**************************        End set params        ********************************/ 


	/***************************** Initialise problems variable here ***********************************/
	VectorXd x(4) , y(4) , z(1) , yaw(1) ,t(4) , corridors(1);				//Corridors size should be x.size() -1
	//get_uniform_helix(5000,300,x,y,z,t);
	//get_lituus_spiral(20 , 20 , x , y , t );
	//get_fermat_spiral(20, 50 , x,y,t);
	//get_circle_trajectory(10 , 30.0 , 1000.0 , x , y , t);
	x << 0.0 , 1.0 , 1.0 , 0.0 ;
	y << 0.0 , 0.0, 2.0 , 2.0  ;
	t << 0.0 , 0.5 , 1.5 , 2.5;
	//t = VectorXd(2);
	//t << 0.0 , 2.5 ;
	//corridors << 0.0 , 0.05  , 0.0 ;

	/***************************************************************************************************/

	/***************************** Initialize Constraint 			*************************************/
	vector<min_snap::ConstraintValue> x_const[x.size()] , y_const[y.size()] , z_const[z.size()] , yaw_const[yaw.size()];

	//No contraint initialization to initialize the arrays
	std::fill_n(x_const , x.size() , vector<min_snap::ConstraintValue>());
	std::fill_n(y_const , y.size() , vector<min_snap::ConstraintValue>());
	std::fill_n(z_const , z.size() , vector<min_snap::ConstraintValue>());
	std::fill_n(yaw_const , yaw.size() , vector<min_snap::ConstraintValue>());

	vector<min_snap::ConstraintValue> zero_constraint;
	min_snap::ConstraintValue value;
	//Initial v = 0
	value.bndl = 0;
	value.bndu = 0;
	value.bndl_valid = true;
	value.bndu_valid = true;
	value.derive_order = 1;
	zero_constraint.push_back(value);
	//Initial a = 0;
	value.derive_order = 2;
	zero_constraint.push_back(value);

	//We set inital speed and acceleration equal to zero for all components
	x_const[0] = zero_constraint;
	x_const[x.size()-1] = zero_constraint;

	y_const[0] = zero_constraint;
	y_const[y.size()-1] = zero_constraint;

	z_const[0] = zero_constraint;
	z_const[z.size()-1] = zero_constraint;

	/*****************************************************************************************************/


	//Wrap everything in waypoint message that will be used by the Solution class to compute solution
	min_snap::WayPoints waypoints;

	for(int i=0 ; i< x.size() && x.size()>1 ; i++){
		min_snap::ConstraintAxis x_axis;
		x_axis.axis = x(i);
		x_axis.constraints = x_const[i];
		waypoints.x.push_back(x_axis);
	}

	for(int i=0 ; i< y.size() && y.size()>1 ; i++){
		min_snap::ConstraintAxis x_axis;
		x_axis.axis = y(i);
		x_axis.constraints = y_const[i];
		waypoints.y.push_back(x_axis);
	}
	for(int i=0 ; i< z.size() && z.size()>1 ; i++){
		min_snap::ConstraintAxis x_axis;
		x_axis.axis = z(i);
		x_axis.constraints = z_const[i];
		waypoints.z.push_back(x_axis);
	}
	for(int i=0 ; i< yaw.size() && yaw.size()>1 ; i++){
		min_snap::ConstraintAxis x_axis;
		x_axis.axis = yaw(i);
		x_axis.constraints = yaw_const[i];
		waypoints.yaw.push_back(x_axis);
	}
	for(int i=0 ; i< t.size() && t.size()>1 ; i++){
		waypoints.t.push_back(t(i));
	}
	for(int i=0 ; i< corridors.size() && corridors.size()>1 ; i++){
		waypoints.corridors.push_back(corridors(i));
	}

	/************************       Solving the problem           **************************************/
	//Frequency for the solution
	int freq = 1000;
	//Create a solution instance for solving the problem
	clock_t begin = clock();
	min_snap::Solution sol(params , waypoints);
	cout << "solving time = " << double(clock() - begin) / CLOCKS_PER_SEC << endl;
	if (! sol.has_solution()) {
		cout << "NO SOLUTION FOUND !! Check again your variables OR constraints ...." << endl;
	}

	min_snap::Trajectory traj;
	sol.get_trajectory(freq , traj);

	//Saving the data
	ofstream results;
	results.open("results.txt", ios::trunc);
	results.close();

	for (vector<min_snap::PVA>::iterator traj_iter=traj.pva.begin(); traj_iter != traj.pva.end() ; traj_iter++ ){
		results.open("results.txt",ios::app);
		results << traj_iter->pos(0) << "," << traj_iter->pos(1) << "," << traj_iter->pos(2) << endl;
		results.close();
	}
	/************************       End solving 				 ***************************************/

	return 0;
}



void get_circle_trajectory(int freq , double duration , double R , VectorXd &x , VectorXd &y , VectorXd &t){
	double t0 = 0;
	x = VectorXd( freq);
	y = VectorXd( freq);
	t = VectorXd( freq);

	int iter = 0;
	while (iter < x.size()){
		x(iter) = R * std::cos(t0);
		y(iter) = R * std::sin(t0);
		t(iter) = iter * (1.0/(x.size()-1)) * duration;
		t0 = PI_2 * (1.0/(x.size()-1)) * (iter+1);
		iter++;
	}
}

void get_fermat_spiral(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &t){
	double t0 = 0;
	x = VectorXd( freq);
	y = VectorXd( freq);
	t = VectorXd( freq);

	int iter = 0;
	while (iter < x.size()){
		t(iter) = iter * (1.0/(x.size()-1)) * duration;
		t0 = t(iter);
		x(iter) = t0 * t0 * std::cos(t0);
		y(iter) = t0 * t0 * std::sin(t0);
		iter++;
	}
}

void get_lituus_spiral(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &t){
	double t0 = 0.1;
	x = VectorXd( freq);
	y = VectorXd( freq);
	t = VectorXd( freq);

	int iter = 0;
	while (iter < x.size()){
		x(iter) = (1.0/t0) * std::cos(t0);
		y(iter) = (1.0/t0) * std::sin(t0);
		t(iter) = iter * (1.0/(x.size()-1)) * duration;
		t0 = t(iter) + 0.15;
		iter++;
	}
}

void get_conical_helix(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &z ,VectorXd &t){
	double t0 = 0;
	x = VectorXd( freq);
	y = VectorXd( freq);
	z = VectorXd( freq);
	t = VectorXd( freq);

	int iter = 0;
	while (iter < x.size()){
		t(iter) = iter * (1.0/(x.size()-1)) * duration;
		t0 = t(iter);
		x(iter) = t0 * std::cos(6*t0);
		y(iter) = t0 * std::sin(6*t0);
		z(iter) = t0;
		iter++;
	}
}

void get_uniform_helix(int freq , double duration , VectorXd &x , VectorXd &y , VectorXd &z ,VectorXd &t){
	double t0 = 0;
	x = VectorXd( freq);
	y = VectorXd( freq);
	z = VectorXd( freq);
	t = VectorXd( freq);

	int iter = 0;
	while (iter < x.size()){
		t(iter) = iter * (1.0/(x.size()-1)) * duration;
		t0 = t(iter);
		x(iter) = std::cos(6*t0);
		y(iter) = std::sin(6*t0);
		z(iter) = t0;
		iter++;
	}
}