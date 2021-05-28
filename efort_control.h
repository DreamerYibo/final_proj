#pragma once

// long int zero_position[18] = {-38852, 1005654, 243343, -125735, -186815, 310557, -72770, 67622, 255432, -109501, -7436, -1179597, -101830, -1794918, -1732384, 137827, 140254, -381659};
// long int incpdeg[18] = {-53521, 65238, -59738, -28017, -25486, -15969, -53521, 65238, -59738, -28017, -25486, -15969, -55324, 55706, -55708, -31147, -26970, 26970};

#include <windows.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <vector>
#include <valarray>
#include <iomanip>
#include "joint_6dofPublisher.h"
#include "joint_6dofSubscriber.h"
#include <Eigen/Dense>
#include "EfortKine.h"

#include "advmotdrv.h"
#include "General.h"
#include "AdvMotApi.h"

#define _USE_MATH_DEFINES
#include <math.h>

const long int zero_position[18] = {-38852, 1005654, 243343, -126753, 1783177, 290672, -72770, 67622, 255432, -109501, -7436, -1179597, -101830, -1794918, -1732384, 137827, 140254, -16030132};
const long int incpdeg[18] = {-53521, 65238, -59738, -28017, -25486, -15969, -53521, 65238, -59738, -28017, -25486, -15969, -55324, 55706, -55708, -31147, -26970, -14474};

// extern const double pi = M_PI;
const double samples_rad_ratio = floor(512.0 / pi); // should move at most 180 degree at each planning scene.

long int deg2inc(double deg, int i);
long int rad2inc(double rad, int i);
double inc2rad(long int inc, int i);

struct AxisProperties
{
    double AxMaxAcc = 20000;
    double AxMaxVel = 20000;
    double AxMaxDec = 20000;
    double AxVelLow = 2000;
    double AxVelHigh = 4000;
    double AxAcc = 3000;
    double AxDec = 3000;
    double AxJerk = 0;
};

extern bool continue_thr1;

Vec6d read_joint_pos(const HAND *axis_hand_first, std::string robot_name); // output unit:rad. Convert encoder's value to rad.
bool set_axis_properties(const HAND *axis_hand, const AxisProperties &axisprop);
bool close_6_axises(HAND *axis_hand);
void out_rel_pos(const std::vector<HAND> &axis_hand, const std::vector<F64> &init_pos);
int ReadnSet_PTdata_fromtxt(const HAND &m_Axishand, const std::string &Filename, int amount);
void GetAxisStateStr(std::string &str, U16 state_num);
void Read_target_jointpos_sets(std::vector<Vec6d> &target_jointpos_cont, const std::string &Filename);
void preview_PTmove(joint_6dofPublisher &mypub, std::vector<Vec6d> &traj_planning_result_cont, PlanParam &param, std::string mode);
void set_PTdata(PlanParam &param, std::vector<Vec6d> &traj_planning_result_cont, HAND *m_Axishand, std::string robot_name);