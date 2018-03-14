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

#ifndef __MIN_SNAP_H_INCLUDED__
#define __MIN_SNAP_H_INCLUDED__

#include <vector>
#include <limits>
#include <utility>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

#define MAX_REAL_VALUE std::numeric_limits<double>::max()

#ifndef SNAP_POSITION_ORDER
#define SNAP_POSITION_ORDER 4
#endif

#ifndef SNAP_YAW_ORDER
#define SNAP_YAW_ORDER 2
#endif

#ifndef MAX_DIM
#define MAX_DIM 4
#endif

#define N_pos (SNAP_POSITION_ORDER+1)*2
#define N_yaw (SNAP_YAW_ORDER+1)*2

#define DEFAULT_PARAMS (min_snap::solver_param){.derive_order=SNAP_POSITION_ORDER , .nc=8 , .epsilon=1e-4 , .minAlpha = 1e-2, .init_alpha=0.9, .min_scale_duration=1.0,.verbose=false}


namespace min_snap
{

	/*
	*	Structure for user constraints on a specific waypoint
	*	bndl_valid = false if the lower bound have to be ignored (-inf) and True if it has to be considered
	*	idk for bndu_valid
	*	label store the label of this variable --> can be x , y , z or yaw ....
	*	derive order = is it velocity (1) , acceleration(2) , jerk(3) etc ...
	*	bndl and bndu stores the lower bound or the upper bound if they are valid
	*	SO -->   bndl <= c <= bndu with c the derive_order'th of the current variable
	*/
	struct ConstraintValue_ {
		bool bndl_valid;
		bool bndu_valid;
		unsigned int  label;
		unsigned int derive_order;
		double bndl;
		double bndu;
	};

	typedef struct ConstraintValue_ ConstraintValue;

	/*
	*	ConstraintAxis stores the current waypoint and all the constraints applied to it
	*	axis = current waypoint value on a specified axis
	*	constraints = list of constraints applied to this waypoint. cf Constraint value for more informations
	*/
	struct ConstraintAxis_ {
		double axis;
		std::vector<ConstraintValue> constraints;
	};

	typedef struct ConstraintAxis_ ConstraintAxis;

	/*
	*	Stores position , velocity , acceleration , yaw and yaw_rate.
	*	Definition is pretty more explicit
	*/
	struct PVA_ {
		Eigen::Vector3d pos;
		Eigen::Vector3d vel;
		Eigen::Vector3d acc;
		double yaw;
		double yaw_rate;
	};

	typedef struct PVA_ PVA;

	/*
	*	Store the final trajectory returned by the minimum snap algorithm
	*	pva : llist of pva at a  pre defined frequency
	*	wait_freq : the frequency of each point of the path
	*/
	struct Trajectory_ {
		std::vector<PVA> pva;
		std::vector<unsigned int> wait_freq;
	};

	typedef struct Trajectory_ Trajectory;

	/*
	*	Waypoints store the user waypoints and all the constraints applied to each waypoint
	*	Structure definition is pretty more explicit
	*/
	struct Waypoints_ {
		std::vector<ConstraintAxis> x;
		std::vector<ConstraintAxis> y;
		std::vector<ConstraintAxis> z;
		std::vector<ConstraintAxis> yaw;

		std::vector<double> t;
		std::vector<double> corridors;
	};

	typedef struct Waypoints_ WayPoints;

	/*
	*	Stores some params for the solver
	*	This can will be used by the solver
	*/
	struct solver_param_ {

		//param for the problem
		int derive_order;			//For modelization problem , we take the polynomial degree = 2 * (derive_order)
		int nc;						//Number of intermediate point for each corridors constraints

		//Gradient descent solver parameter
		double epsilon;				//Relative error for the cost function when using gradient descent
		double minAlpha;			//Min coefficient on thee descent direction before stopping
		double init_alpha;			//Initial coefficient for the nez descent direction
		
		double min_scale_duration;	//Positive Minimum segment duration for numerical stability || Inversibility of M matrix. Preferable to set a value gbigger than 1 but not too big (let's say 10)

		bool verbose;				//True if want to see calculation details				
	};

	/*
	*	Scale version of the time furnished by the user
	*	WHY ? for stability reason we have to be able to inverte the M matrix and this require the minimum segment duration to be at lease
	*	let's say 1.
	*/
	struct new_time_scale_ {
		Eigen::VectorXd t;
		double scale_factor;
		double t_init;
	};


	typedef struct new_time_scale_ new_time_scale;
	typedef struct solver_param_ solver_param;
	typedef Eigen::Triplet<double> T;
	typedef std::pair<T,T> bnd_interval;

	class Solution {

	public:
		Solution(const solver_param params_ , const WayPoints &waypoints);
		bool get_trajectory(const int frequency, Trajectory &traj);
		bool has_solution();
		void get_solution(double m_t , PVA &pva);
	private:
		int M;
		int axis_number;
		double trajectory_duration;
		Eigen::VectorXd solution;
		Eigen::VectorXd yaw;
		std::string axis_label[MAX_DIM];
		Eigen::VectorXd t;
		new_time_scale t_scale;
		bool solve_succeed;

		void get_solution(double m_t, int curr_seg, PVA &pva);
	};
}

bool compute_min_snap_default(const min_snap::solver_param &params, const Eigen::MatrixXd &x, min_snap::new_time_scale &time_scale,
								const std::vector<min_snap::ConstraintValue> constraints[],
									const Eigen::VectorXd &corridors,Eigen::VectorXd &sol);

bool compute_min_snap_optimal_segment_times(const min_snap::solver_param &params,const Eigen::MatrixXd &x, double t_init, double t_end,
												const std::vector<min_snap::ConstraintValue> constraints[],
													const Eigen::VectorXd &corridors, Eigen::VectorXd &sol_x , min_snap::new_time_scale &time_scale);

bool compute_min_snap_iter_corridors(bool optimal_segment,const min_snap::solver_param &params,Eigen::MatrixXd &x,min_snap::new_time_scale &time_scale,
										const std::vector<min_snap::ConstraintValue> constraints[],
												Eigen::VectorXd &corridors, Eigen::VectorXd &sol);


#endif //__MIN_SNAP_H_INCLUDED__