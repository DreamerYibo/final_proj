#include <iostream>
#include <Eigen/Dense>
#include <valarray>
#include <algorithm>
#include <iterator>
#include <vector>
#include <list>

#define _USE_MATH_DEFINES
#include <math.h>

struct PlanParam
{
    double t0;
    double tf;
    unsigned int samples_num;
};

typedef Eigen::Matrix<double, 6, 1> Vec6d;
class EfortRobo
{
public:
    EfortRobo(){};
    EfortRobo(const std::string type);
    ~EfortRobo(){};
    Vec6d x; // theta. Joint pos. UNIT: rad
    Eigen::Matrix<double, 4, 4> forward_kine(Vec6d theta);
    std::vector<Vec6d> inv_kine(Eigen::Matrix<double, 4, 4> &kn_T0_6, std::string mode = "default", Vec6d target_joint_pos = Vec6d::Zero()); // if mode == "search". This fcn will output
                                                                                                //only one sol which is the nearest one to the target.
    void line_traj_planning(std::vector<Vec6d> &result_cont, Vec6d &init_joint_pos, Eigen::Vector3d translation, PlanParam &param);
    void joint_space_planning(std::vector<Vec6d> &result_cont, Vec6d &init_joint_pos, Vec6d &target_joint_pos, PlanParam &param);
    
private:
    std::string robo_type;
    double L1, L2, L3, L4, L5, L8;
    Eigen::Matrix<double, 4, 4> T_temp; //variable to be returned
    Eigen::Matrix<double, 4, 4> &T0_3(double x1, double x2, double x3);
    Eigen::Matrix<double, 4, 4> &T0_4();
    Eigen::Vector3d T0_4_pos(double x1, double x2, double x3); //helper fcn
    double sign(double num);
};

std::vector<double> linspace(double start, double end, unsigned int amount_of_points);
