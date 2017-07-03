#include <iostream>
#include <math.h>
#include "min_snap.h"
#include "optimization.h"
#include <functional>

using namespace std;
using namespace Eigen;

void generate_pow(double t, VectorXd &out);
void generate_derivatives(double t , MatrixXd &out);
void generate_cost_matrix_1d(int N, int derive_order,const VectorXd &t, vector<T> &H);
bool generate_deriv_order_constraints(int N,int derive_order , const MatrixXd &x,const VectorXd &t,
	const vector<vector< min_snap::ConstraintValue> > &constraints, vector<VectorXd> &A, vector<T> &b , vector<int> &rel_a_b);
bool generate_corridors(int N , int nc , const MatrixXd &x,const VectorXd &t,const VectorXd &corridors,
	vector<T> &A, vector<T> &b , vector<int> &rel_a_b);
double lagrange_solve(const solver_param &params , int lin_const ,const int M , const int dim_, const vector<T> &H , const vector<T> &A , const vector<T> &b ,VectorXd &sol);
double alglib_solve(const solver_param &params,const int M , const int dim_,const vector<T> &H , const vector<T> &A , const vector<T> &b , const vector<int> &rel_a_b, VectorXd &sol);

void generate_pow(double t , VectorXd &out){
	out(0) = 1;
	for(int i=0; i <out.size()-1;i++){
		out(i+1) = out(i)*t;
	}
}

void generate_derivatives(double t, MatrixXd &out){
	VectorXd row_0(out.cols());
	generate_pow(t,row_0);
	out.row(0) =  row_0;
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
* Note that the global cost matrix a Diagonal Matrix with this generated 1d cost matrix as diagonal block.
* Basically 
*					Hs(x|y|z|yaw) (i,j) =  Hs(x|y|z|yaw) (j,i) = integral (der(t^i , derive_order) * der(t^j ,derive_order) , ts , ts+1)
* Finally 
*					H = diag(Ho , H1 , ... , Hm) where m is number of segments
*/
void generate_cost_matrix_1d(int N , int derive_order,const VectorXd &t, vector<T> &H){
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
	//Non zeros coefficient of lower triangle H matrix
	for (int s = 0; s<M ;s++){
		coeff_iter = 0;
		for (int k=0; k<N; k++){
			for (int p=0; p<=k; p++){
				if( k>=derive_order && p>=derive_order){
					int pow_val =  k + p - 2*derive_order +1;
					H.push_back(T(s*N+k,s*N+p,(pow(t(s+1),pow_val) - pow(t(s),pow_val))*compute_coeff[coeff_iter]));
				}
				coeff_iter++;
			}
		}
	}
}

bool generate_corridors(int N , int nc, const MatrixXd &x,const VectorXd &t, const VectorXd &corridors,
	vector<T> &A, vector<T> &b , vector<int> &rel_a_b){

	int available_corridor = 0;
	int M = x.rows() - 1;

	for (int m = 0; m < M; m++){
		if (corridors(m) <=0){
			continue;
		}
		available_corridor++;
		VectorXd next_point = x.row(m+1);
		VectorXd current_point = x.row(m);
		VectorXd norm_vect = (next_point - current_point).normalized();
		VectorXd b_val = current_point - ((current_point.dot(norm_vect))*norm_vect);
		for (int j = 1 ; j<= nc; j++){
			VectorXd poly_vec = VectorXd::Zero(N);
			generate_pow(t(m) + (((double) j) * (t(m+1) - t(m))/(1+nc)),poly_vec);
			for (int axis =0; axis<x.cols();axis++){
				for(int a_iter = 0 ; a_iter <N ; a_iter++){
					double value = poly_vec[a_iter]*(1-norm_vect(axis)*norm_vect(axis));
					if (value != 0.0){
						A.push_back(T(rel_a_b.size(),axis*N*M + m*N + a_iter,value));
						A.push_back(T(rel_a_b.size()+1,axis*N*M + m*N + a_iter,value));
					}
				}
				for (int axis_sub = 0;axis_sub<x.cols(); axis_sub++){
					if(axis_sub != axis){
						for(int a_iter = 0 ; a_iter <N ; a_iter++){
							double value = -poly_vec[a_iter]*norm_vect(axis_sub)*norm_vect(axis);
							if(value != 0){	
								A.push_back(T(rel_a_b.size(),axis_sub*N*M + m*N + a_iter,value));
								A.push_back(T(rel_a_b.size()+1,axis_sub*N*M + m*N + a_iter,value));
							}
						}
					}
				}
				double sup_value = b_val(axis) + (corridors(m));
				double inf_value =b_val(axis) - (corridors(m));
				if (sup_value != 0){
					b.push_back(T(rel_a_b.size(),0,sup_value));
				}
				rel_a_b.push_back(min_snap::ConstraintValue::INF);
				if (inf_value !=0){
					b.push_back(T(rel_a_b.size(),0,inf_value));
				}
				rel_a_b.push_back(min_snap::ConstraintValue::SUP);
			}

		}
	}
	return (available_corridor != 0);
}

//TODO --> preferable to put eps for zero comparison
bool generate_deriv_order_constraints(int N, int derive_order, const MatrixXd &x,const VectorXd &t,
	const vector< vector<min_snap::ConstraintValue> > &constraints,
	vector<T> &A, vector<T> &b , vector<int> &rel_a_b){

	int M = x.rows() -1;
	bool contain_inequality = false;
	//Order 0 constraints such as position, yaw constraints
	vector< vector<min_snap::ConstraintValue> >::const_iterator constraints_iterator = constraints.begin();

	vector<min_snap::ConstraintValue> last_constraint;

	MatrixXd start_der_vect(derive_order+1,N);
	generate_derivatives(t(0),start_der_vect);
	for (int m=0;m<M;m++){
		last_constraint = *constraints_iterator;
		constraints_iterator++;

		MatrixXd end_der_vect(derive_order+1,N);
		generate_derivatives(t(m+1),end_der_vect);
		for (int axis = 0;axis<x.cols(); axis++){
			for(int a_iter =0; a_iter< N ; a_iter++){
				if (start_der_vect(0,a_iter) != 0){
					A.push_back(T(rel_a_b.size(),axis*N*M+m*N+ a_iter,start_der_vect(0,a_iter)));
				}
			}
			if (x(m,axis) != 0){
				b.push_back(T(rel_a_b.size(),0,x(m,axis)));
			}
			rel_a_b.push_back(min_snap::ConstraintValue::EQUAL);
			for(int a_iter =0; a_iter< N ; a_iter++){
				if (end_der_vect(0,a_iter) != 0){
					A.push_back(T(rel_a_b.size(),axis*N*M+m*N+ a_iter,end_der_vect(0,a_iter)));
				}
			}
			if (x(m+1,axis) != 0){
				b.push_back(T(rel_a_b.size(),0,x(m+1,axis)));
			}
			rel_a_b.push_back(min_snap::ConstraintValue::EQUAL);
		}

		//user constraints and initialisation to false for security
		bool already_constrains[derive_order];
		for (int d = 0;d<derive_order;d++){
			already_constrains[d] = false;
		}
		
		if(!last_constraint.empty()){
			for (vector<min_snap::ConstraintValue>::const_iterator c = last_constraint.begin() ; c!= last_constraint.end();++c){
				for(int a_iter =0; a_iter< N ; a_iter++){
					if (start_der_vect(c->derive_order,a_iter) != 0){
						A.push_back(T(rel_a_b.size(),c->label*N*M+m*N+ a_iter,start_der_vect(c->derive_order,a_iter)));
					}
				}
				if(c->value != 0){
					b.push_back(T(rel_a_b.size(),0,c->value));
				}
				rel_a_b.push_back(c->comparison);
				if(c->comparison != min_snap::ConstraintValue::EQUAL){
					contain_inequality = true;
				}
			}
		}
		if(!(constraints_iterator->empty())){
			for (vector<min_snap::ConstraintValue>::const_iterator c = constraints_iterator->begin() ; c!= constraints_iterator->end();++c){
				for(int a_iter =0; a_iter< N ; a_iter++){
					if(end_der_vect(c->derive_order,a_iter) != 0){
						A.push_back(T(rel_a_b.size(),c->label*N*M+m*N+ a_iter, end_der_vect(c->derive_order,a_iter)));
					}
				}
				if(c->value != 0){
					b.push_back(T(rel_a_b.size(),0,c->value));
				}
				rel_a_b.push_back(c->comparison);
				if(c->comparison != min_snap::ConstraintValue::EQUAL){
					contain_inequality = true;
				}
				already_constrains[c->derive_order-1] = true;
			}
		}
		//Handle non user constraints --> continuity for the intermediate points
		for(int i = 1; i<= derive_order; i++){
			if (already_constrains[i-1] || m>= M-1){
				continue;
			}
			for (int axis = 0 ; axis<x.cols() ; axis++){
				for(int a_iter =0; a_iter< N ; a_iter++){
					if( end_der_vect(i,a_iter) != 0){
						A.push_back(T(rel_a_b.size(),axis*N*M + m*N+ a_iter, end_der_vect(i,a_iter)));
						A.push_back(T(rel_a_b.size(),axis*N*M + (m+1)*N + a_iter, -end_der_vect(i,a_iter)));
					}
				}
				rel_a_b.push_back(min_snap::ConstraintValue::EQUAL);
			}
		}
		start_der_vect = end_der_vect;
	}
	return contain_inequality;
}


double lagrange_solve(const solver_param &params , int lin_const ,const int M , const int dim_,const vector<T> &H , const vector<T> &A , const vector<T> &b , VectorXd &sol){
	//Solve the problem M.Y = X 
	// Where M = [H  t(A)]
	//			 [A  0]
	//and X = [0]  with Y = [var_sy]
	//	      [b]			[lambda]
	//vector<T> M_vec;

	//TODO use sparse matrix to solve the problem
	//First tried failed --> maybe numeric integration
	int dim_H = dim_*params.N*M;
	MatrixXd M_mat = MatrixXd::Zero(dim_H+lin_const , dim_H + lin_const);

	for( vector<T>::const_iterator h_sub = H.begin(); h_sub != H.end() ; h_sub++){
		for (int i =0 ; i< dim_ ; i++){
			M_mat(i*params.N *M + h_sub->row(),i*params.N *M + h_sub->col()) = h_sub->value();
			M_mat(i*params.N *M + h_sub->col(),i*params.N *M + h_sub->row()) = h_sub->value();
		}
	}
	for( vector<T>::const_iterator a_sub = A.begin(); a_sub != A.end() ; a_sub++){
		M_mat(dim_H+a_sub->row(),a_sub->col()) = a_sub->value();
		M_mat(a_sub->col(),dim_H+a_sub->row()) = a_sub->value();
	}

	VectorXd X = VectorXd::Zero(dim_H + lin_const);
	for( vector<T>::const_iterator b_sub = b.begin(); b_sub != b.end() ; b_sub++){
		X(dim_H+b_sub->row()) = b_sub->value();
	}
	VectorXd Y = M_mat.colPivHouseholderQr().solve(X);
	sol = Y.head(dim_H);
	double cost = sol.transpose()*M_mat.block(0,0,dim_H,dim_H)*sol; // X'*H*X = Y' * X;
	return cost/2;
}


double alglib_solve(const solver_param &params,const int M , const int dim_,const vector<T> &H , const vector<T> &A , const vector<T> &b , const vector<int> &rel_a_b, VectorXd &sol){
	//Solve the optimizaton problem using alglib library and DENSE-AUL solver
	//The problem is the following:
	//		min 0.5 t(X)HX
	//			AX o b

	alglib::sparsematrix H_alg;
	alglib::sparsematrix A_b_alg;
	alglib::real_1d_array s ;
	alglib::integer_1d_array comparison;

	//Initialize H matrix --> Just need the Lower Triangle since H is symmetric
	int dim_H = params.N * M * dim_ ; 
	sparsecreate(dim_H , dim_H , H.size()*dim_,H_alg);
	MatrixXd H_mat = MatrixXd::Zero(dim_H, dim_H);
	for( vector<T>::const_iterator h_sub = H.begin() ; h_sub != H.end() ; h_sub++){
		for(int i=0 ; i<dim_ ; i++){
			sparseset(H_alg,i*params.N*M + h_sub->row(),i*params.N*M +h_sub->col(),h_sub->value());
			H_mat(i*params.N*M + h_sub->row(),i*params.N*M +h_sub->col()) = h_sub->value();
			H_mat(i*params.N*M +h_sub->col(),i*params.N*M + h_sub->row()) = h_sub->value();
		}
	}

	//Initialize the linear constraints matrix
	sparsecreate(rel_a_b.size(),dim_H+1,A.size()+b.size(),A_b_alg);
	for( vector<T>::const_iterator a_sub = A.begin() ; a_sub != A.end() ; a_sub++){
		sparseset(A_b_alg,a_sub->row(),a_sub->col(),a_sub->value());
	}
	for( vector<T>::const_iterator b_sub = b.begin() ; b_sub != b.end() ; b_sub++){
		sparseset(A_b_alg,b_sub->row(),dim_H,b_sub->value());
	}
	//initialize comparison for linear constraints and scaler --> default value used is one
	s.setlength(dim_H);
	for(int i=0 ; i<dim_H ; i++){
		s(i) = 1.0;
	}
	////initialize comparison value for linear constraints
	comparison.setlength(rel_a_b.size());
	int comp_ind = 0;
	for (vector<int>::const_iterator rel = rel_a_b.begin(); rel!= rel_a_b.end() ; rel++){
		if (*rel == min_snap::ConstraintValue::EQUAL){
			comparison(comp_ind) = 0;
		}else if(*rel == min_snap::ConstraintValue::INF){
			comparison(comp_ind) = -1;
		}else if(*rel == min_snap::ConstraintValue::SUP){
			comparison(comp_ind) = 1;
		}
		comp_ind++;
	}
	//solver and report variables
	alglib::real_1d_array res;
	alglib::minqpstate state;
	alglib::minqpreport rep;
	//create the solver
	minqpcreate(dim_H,state);
	minqpsetquadratictermsparse(state,H_alg,false);
	minqpsetlcsparse(state,A_b_alg,comparison,rel_a_b.size());
	minqpsetscale(state,s);
	//Choose the solver -> dense aul solver for important number of linear contraints
	//cout << "Linear system size = " << rel_a_b.size() << endl;
	if(!params.use_denseaul){
		minqpsetalgobleic(state,params.epsg,params.epsf,params.epsx,0);
	}else if(rel_a_b.size() > BLEIC_SOLVER_CONSTRAINTS_SIZE ){
		minqpsetalgodenseaul(state ,params.eps , params.rho ,params.outer_iter);
	} else {
		minqpsetalgobleic(state,params.epsg,params.epsf,params.epsx,0);
	}
	//minqpsetalgodenseaul(state ,params.eps , params.rho ,params.outer_iter);
	//Solve the problem and store in res the solution
	minqpoptimize(state);
	minqpresults(state, res , rep);

	if(rep.terminationtype == -4){
		//cout << " solver found unconstrained direction of negative curvature (function is unbounded from below  even  under  constraints),  no  meaningful minimum can be found" << endl;
		return MAX_REAL_VALUE;
	}
	if (rep.terminationtype == -3){
		//cout << "inconsistent constraints (or, maybe, feasible point is too hard to find). If you are sure that constraints are feasible, try to restart optimizer with better initial approximation" << endl;
		return MAX_REAL_VALUE;
	}
	if(rep.terminationtype ==-1){
		cout << " solver error " << endl;
		return MAX_REAL_VALUE;
	}
	if(rep.terminationtype == 5){
		//cout << "Max Iterations steps was taken" << endl;
	}
	if(rep.terminationtype == 7){
		//cout << " stopping conditions are too stringent, further improvement is impossible, X contains best point found so far " << endl;
	}

	//cout << " InnerIterationsCount = " << rep.inneriterationscount << endl;
	//cout << " OuterIterationsCount = " << rep.outeriterationscount << endl;
	//cout << " number of matrix-vector products = " << rep.nmv << endl;
	//cout << " number of Cholesky decomposition = " << rep.ncholesky << endl;
	//Copying the solution in output argument
	for(int i=0; i<dim_H;i++){
		sol(i) = res(i);
	}
	double cost = sol.transpose()*(H_mat*sol);
	return cost/2;
}

void compute_min_snap_default (const solver_param &params, const MatrixXd &x,const VectorXd &t,
	const vector< vector<min_snap::ConstraintValue> > &constraints,const VectorXd &corridors,VectorXd &sol , double &cost){

	if(x.rows() != t.size()){
		cout << "waypoints, time, and list have not the same size OR differs from M !" <<endl;
		cost = MAX_REAL_VALUE;
		return ;
	}
	if(corridors.size() != x.rows()-1 ){
		cout << "inconsistent value M and corridors list size constraints ! Fill it with 0 if no use ..." << endl;
		cost = MAX_REAL_VALUE;
		return ;
	}
	if(constraints.size() != x.rows() ){
		cout << "inconsistent constraints size value and X list size ! Fill it with empy constraints if no use ..." << endl;
		cost = MAX_REAL_VALUE;
		return ;
	}
	int M = x.rows() - 1;
	int dim_ = x.cols();
	//sparse Cost vector for each axis is the same , sparce linear constraints vector , comparison vector
	vector<T> H_m , A , b ;
	vector<int> rel_a_b;

	//pre allocation of H_m size = max size that can possibly take H_m (base on his symmetric spec)
	H_m.reserve(((params.N*(params.N+1))/2)*M);
	//pre alloc of A , b and rel_a_b --> simple estimation
	A.reserve(2*(M+1)*dim_ + M*params.derive_order);
	b.reserve(2*(M+1)*dim_ + M*params.derive_order);
	rel_a_b.reserve(2*(M+1)*dim_ + M*params.derive_order);
	//generate 1 axis = N*M , N*M  cost Matrix
	generate_cost_matrix_1d(params.N,params.derive_order,t,H_m);
	//generate linear constraints except from corridors constraints
	bool has_ineq = generate_deriv_order_constraints(params.N,params.derive_order,x,t,constraints,A,b,rel_a_b);
	//Use the same instance for the generation of corridors cosntraints
	if(x.cols() > 1){
		has_ineq = has_ineq || generate_corridors(params.N,params.nc,x,t,corridors,A,b,rel_a_b);
	}//No need to generate corridors since it doesn't make sense in 1D
	//Depending on if the system contains inequality or not it may be useful to use lagrange multiplier to go faster
	if(params.use_alglib){
		//cout << "ALGLIB solver MANUALLY chosen ..... " << endl;
		cost = alglib_solve(params,M,dim_,H_m,A,b,rel_a_b,sol);
		return ;
	}
	if (!has_ineq ){
		//cout << "Lagrange Multiplier solver method chosen ...." << endl;
		cost = lagrange_solve(params,rel_a_b.size(),M,dim_,H_m,A,b,sol);
	}else{
		//cout << "ALGLIB solver chosen ..... " << endl;
		cost =  alglib_solve(params,M,dim_,H_m,A,b,rel_a_b,sol);
	}
}

double compute_min_snap_optimal_segment_times(const solver_param &params, const MatrixXd &x,double t_init,double t_end,
	const vector< vector<min_snap::ConstraintValue> > &constraints,const VectorXd &corridors, VectorXd &sol_x, VectorXd &sol_t){
	
	double cost;
	auto func_to_minimized = [&](const alglib::real_1d_array &t , double &func , void *ptr){
		VectorXd temps(x.rows());
		temps(0) = t_init;
		for(int i=0 ; i<t.length(); i++){
			temps(i+1) = temps(i) + t(i);
		} 
		compute_min_snap_default(params, x ,temps,constraints, corridors,sol_x,cost);
		func = cost;
	};

	//Compute initial guess based on the current distance between point
	VectorXd normVect(x.rows()-1);
	for(int i= 1; i< x.rows() ; i++){
		normVect(i-1) = (x.row(i)-x.row(i-1)).norm();
	}
	double sum_nv = normVect.sum();

	//use alglib to solve the problem 
	//initial estimation
	alglib::real_1d_array t;
	t.setlength(x.rows()-1);
	for(int i = 0 ; i< x.rows()-1;i++){
		t(i) = ((normVect(i))/sum_nv) * (t_end - t_init);
	}

	//bounds
	alglib::real_1d_array bndl , bndu;
	bndl.setlength(x.rows()-1);
	bndu.setlength(x.rows()-1);
	for(int i = 0 ; i<x.rows()-1 ; i++){
		bndl(i) = 0.0;
		bndu(i) = MAX_REAL_VALUE;
	}

	//linear constraints
	alglib::real_2d_array c;
	alglib::integer_1d_array ct;
	c.setlength(1,x.rows());
	ct.setlength(1);
	//Cond sum(Ti) = t_end - t_init
	for(int i = 0; i<x.rows() -1; i++){
		c(0,i) = 1.0;
	}
	c(0,x.rows()-1) = t_end -t_init;
	ct(0) = 0;

	//object and function call for the solver
	alglib::minbleicstate state;
	alglib::minbleicreport rep;

	minbleiccreatef(t ,params.diffstep,state);
	minbleicsetbc(state ,bndl , bndu);
	minbleicsetlc(state,c,ct);
	minbleicsetcond(state,params.epsg,params.epsf,params.epsx,params.outer_iter);
	minbleicoptimize(state, func_to_minimized);
	minbleicresults(state,t , rep);

	sol_t(0) = t_init;
	for(int i =0; i< x.rows()-1 ; i++){
		sol_t(i+1) = sol_t(i) + t(i);
	}

	if(rep.terminationtype == -8){
		cout << "internal integrity control detected  infinite  or  NAN  values  in function/gradient" <<endl;
		return MAX_REAL_VALUE;
	}else if(rep.terminationtype == -7){
		cout << "gradient verification failed." <<endl;
	}else if(rep.terminationtype == -3){
		cout << "inconsistent constraints. Feasible point is either nonexistent or too hard to find" << endl;
	}
	cout << "IterationsCount = " << rep.iterationscount << endl;
	cout << "Number of gradient evaluated = " << rep.nfev << endl;
	cout << "Error in Equality constraints = " << rep.debugeqerr << endl;

	return cost;
}