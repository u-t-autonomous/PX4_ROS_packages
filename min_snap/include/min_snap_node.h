#include "min_snap.h"
#include "min_snap/Trajectory.h"
#include "geometry_msgs/Point.h"

#define SNAP_POSITION_ORDER 4
#define SNAP_YAW_ORDER 2
#define DEFAULT_PARAMS (solver_param){.eps=1e-9 , .rho=10000 , .outer_iter=10 , .epsg=1e-9 , .epsf=0 , .epsx=0 , .diffstep=1e-6 , .N=6 , .derive_order=SNAP_POSITION_ORDER , .nc=8 , .use_alglib=false, .use_denseaul=false}
#define MAX_DIM 4

class Solution {

public:
	Eigen::VectorXd sol_point;
	Eigen::VectorXd yaw;
	std::string axis_label[MAX_DIM];
	Eigen::VectorXd t;
	Solution(int m_N, int m_M , int m_d);
	void get_trajectory(int frequency, min_snap::Trajectory &traj);
	void get_solution(double m_t, int curr_seg, geometry_msgs::Point &pos,geometry_msgs::Point &vel,geometry_msgs::Point &acc , double &yaw , double &yaw_rate);
private:
	int N;
	int M;
	int d;
};