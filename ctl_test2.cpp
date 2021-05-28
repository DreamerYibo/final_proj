// #include <basetsd.h>
// #include <windef.h>
// #include <winnt.h>
#include <windows.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <vector>
#include <valarray>
#include <iomanip>
#include <Eigen/Dense>
#include "EfortKine.h"
#include "efort_control.h"

#define _USE_MATH_DEFINES
#include <math.h>

bool continue_thr1 = true;

int main()
{
    using std::cin;
    using std::cout;
    using std::endl;

    joint_6dofPublisher mypub;
    EfortRobo robo2("ER20");
    Eigen::MatrixXd joint_input(6, 1);
    joint_input << 1, 2, 3, 4, 5, 6;

    std::vector<Vec6d> traj_planning_result_cont; // clear it before each use of EfortKine::line_traj_planning()

    std::valarray<U32> Ret(6);
    U32 m_ulAxisCount;
    HAND m_Devhand;
    U32 m_DevNum;
    HAND m_Axishand[6];

    std::string temp_str;

    Ret[0] = Acm_GetDevNum(Adv_PCI1203, 0, (&m_DevNum)); // Get M0 DEVNUM
    std::cout << "The M0 device number is " << m_DevNum << std::endl;

    //cin >> temp;
    Ret[0] = Acm_DevOpen(m_DevNum, &(m_Devhand));                              // Open M0 and get its Handle
    Ret[0] = Acm_GetU32Property(m_Devhand, FT_DevAxesCount, &(m_ulAxisCount)); //Get total number of axies of M0
    cout << "Return value for getting the M0 device's axises " << std::hex << Ret[0] << endl;
    cout << "The number of axises is:" << m_ulAxisCount << endl;

    for (int i = 0; i < m_ulAxisCount; i++)
    {
        Ret[i] = Acm_AxOpen(m_Devhand, i, &(m_Axishand[i])); //get axis's handle
    }

    AxisProperties prop3;
    prop3.AxMaxAcc = 300000;
    prop3.AxMaxDec = 300000;
    prop3.AxMaxVel = 200000;
    prop3.AxAcc = 60000;
    prop3.AxDec = 60000;
    prop3.AxJerk = 0;
    prop3.AxVelHigh = 130000;
    prop3.AxVelLow = 2000;

    for (int i = 0; i < m_ulAxisCount; i++) // servo on
    {
        set_axis_properties(&(m_Axishand[i]), prop3);
        Acm_AxResetError(m_Axishand[i]);
        Ret[i] = Acm_AxSetSvOn(m_Axishand[i], 1);
    }

    cout << "Turning on the servo of axis123456. Ret = " << Ret[0] << " " << Ret[1] << "" << Ret[2]
         << "" << Ret[3] << "" << Ret[4] << "" << Ret[5] << endl;

    cout << "Next step? (Y/N)" << endl;
    cin >> temp_str;
    while (temp_str == "N")
    {
        for (int i = 0; i < m_ulAxisCount; i++) // servo on
        {
            Ret[i] = Acm_AxSetSvOn(m_Axishand[i], 1);
            cout << "Turning on the servo of axis123456. Ret = " << Ret[0] << " " << Ret[1] << "" << Ret[2]
                 << "" << Ret[3] << "" << Ret[4] << "" << Ret[5] << endl;
        }
    }

    std::vector<F64> init_pos_vec(6);
    std::vector<HAND> m_Axishand_vec(6);
    PlanParam param; // traj_planning

    for (int i = 0; i < m_ulAxisCount; i++) // put into the container
    {
        m_Axishand_vec[i] = m_Axishand[i];
    }

    if (mypub.init())
    {
        std::vector<Vec6d> joint_pos_target_series;

        Read_target_jointpos_sets(joint_pos_target_series, "robo2_joint_target_series.txt");

        for (int i = 0; i < joint_pos_target_series.size(); i++)
        {
            Vec6d init_joint_pos, target_joint_pos;
            param.tf = 4; // mean ang vel should be lower than 10 degree/s.
            param.t0 = 0;
            param.samples_num = 100;

            if (i == 0)
            {
                init_joint_pos << 0, 0, 0, 0, 0, 0;
            }
            target_joint_pos = joint_pos_target_series[i];

            double max_delta_joint_pos = (target_joint_pos - init_joint_pos).cwiseAbs().maxCoeff();
            param.tf = max_delta_joint_pos / (10.0 * pi / 180.0); // mean ang vel should be lower than 10 degree/s.
            param.t0 = 0;
            param.samples_num = floor(max_delta_joint_pos * samples_rad_ratio); //MUST BE SMALLER THAN 512

            robo2.joint_space_planning(traj_planning_result_cont, init_joint_pos, target_joint_pos, param);
            preview_PTmove(mypub, traj_planning_result_cont, param, "quick");

            // test this rot move
            init_joint_pos = target_joint_pos; // update the init_joint_pos
            traj_planning_result_cont.clear(); // clear the last result
            param.tf = 3;                      // mean ang vel should be lower than 10 degree/s.
            param.t0 = 0;
            param.samples_num = 80; //MUST BE SMALLER THAN 512

            Eigen::Vector3d k_axis = (robo2.forward_kine(init_joint_pos)).block<3, 1>(0, 0); // along end's x frame

            robo2.rotate_traj_planning(traj_planning_result_cont, init_joint_pos, k_axis, 0.5, param);

            preview_PTmove(mypub, traj_planning_result_cont, param, "real_time");

            init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
            traj_planning_result_cont.clear(); // clear the last result

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    set_PTdata(param, traj_planning_result_cont, m_Axishand, "robot2");

    cout << "Start PT move?" << endl;
    cin >> temp_str;
    if (temp_str == "N" || temp_str == "n")
    {
        // close the device

        for (int i = 0; i < 6; i++)
        {
            Acm_AxSetSvOn((m_Axishand[i]), 0); //SVOFF
            Acm_AxClose(&(m_Axishand[i]));     // close axises.
        }

        Ret[0] = Acm_DevClose(&(m_Devhand));
        cout << "The device is closed with return value: " << Ret[0] << endl;
        return 1;
    }

    std::thread thr1(out_rel_pos, m_Axishand_vec, init_pos_vec);

    // 执行 三个轴PT 运动
    // Ret = Acm_AxStartPT(m_Axishand[3], 0);
    Ret = Acm_AxStartAllPT(&(m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6
    cout << "Type in a char to stop moving" << endl;
    cin >> temp_str;

    for (int i = 0; i < 6; i++)
    {
        Acm_AxStopDec(m_Axishand[i]); // stop move.
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait until the axis stops.

    continue_thr1 = 0;
    thr1.join();

    // close the device

    for (int i = 0; i < 6; i++)
    {
        Acm_AxSetSvOn((m_Axishand[i]), 0); //SVOFF
        Acm_AxClose(&(m_Axishand[i]));     // close axises.
    }

    Ret[0] = Acm_DevClose(&(m_Devhand));
    cout << "The device is closed with return value: " << Ret[0] << endl;

    return 0;
}




