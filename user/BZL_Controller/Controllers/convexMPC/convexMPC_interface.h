#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

//#include "common_types.h"

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup
{
  float dt;                                 //（1）时间
  float mu;                                 //（2）摩擦系数
  float f_max;                              //（3）最大力
  int horizon;                              //（4）分段数
};

//更新的数据的结构体
struct update_data_t
{
  float p[3];                               //（1）位置p
  float v[3];                               //（2）速度v
  float q[4];                               //（3）四元数q
  float w[3];                               //（4）角速度w
  float r[12];                              //（5）四脚从COM指向足端向量r
  float yaw;                                //（6）偏航角yaw

  /// Add Begin yuzhiyou 2021-04-13, mod robot state
  float rpy[3];
  /// Add End

  float weights[12];                        //（7）权重优化参数weights
  float traj[12*K_MAX_GAIT_SEGMENTS];       //（8）参考轨迹traj（12）--（欧拉角（3x1），机身位置（3x1），机身角速度（3*1），机身速度（3*1）） *片段数
  float alpha;                              //（9）mpc参数alpha K 式（31）
  unsigned char gait[K_MAX_GAIT_SEGMENTS];  //（10）步态gait
  unsigned char hack_pad[1000];
  int max_iterations;                       //（11）最大迭代次数
  double rho, sigma, solver_alpha, terminate;//（12）jcqp使用参数
  int use_jcqp;                             //（13）使用解法jcqp与否
  float x_drag;                             //（14）z轴方向加速度受x轴方向速度的影响程度x_drag
};

EXTERNC void setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

/// Add Begin yuzhiyou 2021-04-13, mod robot state
EXTERNC void update_problem_data_floats_yzy(float* p, float* v, float* q, float* w,
	                                          float* r, const float* rpy, float* weights,
	                                          float* state_trajectory, float alpha, int* gait);
/// Add End

void update_x_drag(float x_drag);
#endif