#ifndef __MIN_SNAP_H_INCLUDED__
#define __MIN_SNAP_H_INCLUDED__

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "min_snap/ConstraintValue.h"
#include <vector>
#include <limits>

#define MAX_REAL_VALUE std::numeric_limits<double>::max()
#define BLEIC_SOLVER_CONSTRAINTS_SIZE 35

struct solver_param_ {
	//params for the dense aul algorithm
	double eps;
	double rho;
	double outer_iter;
	//params for the bleic algorithm
	double epsg; //condition for the scaled gradient -->subroutines finishes
	double epsf; //The  subroutine  finishes  its work if on k+1-th iteration the  condition  |F(k+1)-F(k)|<=EpsF*max{|F(k)|,|F(k+1)|,1} is satisfied
	double epsx; // condition on the scaled step vector
	double diffstep; //differentiation step for gradient on the axis's
	//param for the problem
	int N;
	int derive_order;
	int nc;
	bool use_alglib;
	bool use_denseaul; //true -> denseaul solver used depending on the number of linear constraints | | false -> use bleic
};
typedef struct solver_param_ solver_param;
typedef Eigen::Triplet<double> T;

void generate_derivatives(double t , Eigen::MatrixXd &out);
void compute_min_snap_default(const solver_param &params, const Eigen::MatrixXd &x,const Eigen::VectorXd &t,
	const std::vector< std::vector<min_snap::ConstraintValue> > &constraints,const Eigen::VectorXd &corridors,Eigen::VectorXd &sol, double &cost);
double compute_min_snap_optimal_segment_times(const solver_param &params,const Eigen::MatrixXd &x, double t_init, double t_end,
	const std::vector< std::vector<min_snap::ConstraintValue> > &constraints,const Eigen::VectorXd &corridors, Eigen::VectorXd &sol_x , Eigen::VectorXd &sol_t);

/*void generate_pow(double t, Eigen::VectorXd &out);
void generate_cost_matrix_1d(int N,int derive_order,const Eigen::VectorXd &t, std::vector<T> &H);
bool generate_deriv_order_constraints(int N, int derive_order , const Eigen::MatrixXd &x,const Eigen::VectorXd &t,
const std::vector< std::vector<min_snap::ConstraintValue> > &constraints, std::vector<T> &A, std::vector<T> &b , std::vector<int> &rel_a_b);
bool generate_corridors(int N, int nc, const Eigen::MatrixXd &x,const Eigen::VectorXd &t,const Eigen::VectorXd &corridors,
	std::vector<T> &A, std::vector<T> &b , std::vector<int> &rel_a_b);
double alglib_solve(const solver_param &params,const int M , const int dim_, const std::vector<T> &H , const std::vector<T> &A , const std::vector<T> &b , const std::vector<int> &rel_a_b, Eigen::VectorXd &sol);
double lagrange_solve(const solver_param &params ,int lin_const, const int M , const int dim_,const std::vector<T> &H , const std::vector<T> &A , const std::vector<T> &b ,Eigen::VectorXd &sol);*/

#endif //__MIN_SNAP_H_INCLUDED__