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
#include "joint_6dofPublisher.h"
#include "joint_6dofSubscriber.h"
#include <Eigen/Dense>
#include "EfortKine.h"

#include "advmotdrv.h"
#include "General.h"
#include "AdvMotApi.h"

#define _USE_MATH_DEFINES
#include <math.h>

long int zero_position[18] = {-38852, 1005654, 243343, -125735, -186815, 310557, -72770, 67622, 255432, -109501, -7436, -1179597, -101830, -1794918, -1732384, 137827, 140254, -381659};
long int incpdeg[18] = {-53521, 65238, -59738, -28017, -25486, -15969, -53521, 65238, -59738, -28017, -25486, -15969, -55324, 55706, -55708, -31147, -26970, 26970};

const double pi = M_PI;

long int deg2inc(double deg, int i)
{
    return (long int)(deg * incpdeg[i]) + zero_position[i];
}

long int rad2inc(double rad, int i)
{
    return (long int)(rad * 180.0 / 3.1415926 * incpdeg[i]) + zero_position[i];
}
double inc2rad(long int inc, int i)
{
    return (((double)(inc - zero_position[i])) / incpdeg[i]) * 3.1415926 / 180.0;
}

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

bool set_axis_properties(const HAND *axis_hand, const AxisProperties &axisprop);
void out_rel_pos(const std::vector<HAND> &axis_hand, const std::vector<F64> &init_pos);
int ReadnSet_PTdata_fromtxt(const HAND &m_Axishand, const std::string &Filename, int amount);
void GetAxisStateStr(std::string &str, U16 state_num);

void preview_PTmove(joint_6dofPublisher &mypub, std::vector<Vec6d> &traj_planning_result_cont, PlanParam &param)
{
    Eigen::MatrixXd joint_input(6, 1);
    double publish_sleep_time = (param.tf - param.t0) / (param.samples_num - 1.0) * 1000;

    for (int i = 0; i < traj_planning_result_cont.size(); i++)
    {
        joint_input = traj_planning_result_cont[i];
        mypub.publish(joint_input, 2);
        // joint_input(1) += 0.1;
        std::this_thread::sleep_for(std::chrono::milliseconds((int)publish_sleep_time));
    }
}

void set_PTdata(PlanParam &param, std::vector<Vec6d> &traj_planning_result_cont, HAND *m_Axishand, std::string robot_name)
{
    double time = 0;
    double delta_time = (param.tf - 0.0) / (param.samples_num - 1.0) * 1000.0; // unit ms;
    double Ret[6];

    int indexoffset = 0; // used to call incpdeg function

    if (robot_name == "robot1")
    {
        indexoffset = 0;
    }
    else if (robot_name == "robot2")
    {
        indexoffset = 6;
    }
    else if (robot_name == "robot3")
    {
        indexoffset = 12;
    }
    else
    {
        std::cout << "Wrong Robot_name!\n";
    }

    for (int i = 0; i < 6; i++)
    {
        Ret[i] = Acm_AxResetPTData(m_Axishand[i]); // 重置 PT 缓存

        for (int j = 0; j < traj_planning_result_cont.size(); j++)
        {
            double move_rel_pulse;                                                                                                        // relative amount of pulses need to move.
            move_rel_pulse = (traj_planning_result_cont[j](i) - traj_planning_result_cont[0](i)) * 180.0 / pi * incpdeg[i + indexoffset]; // robo2. NOTE
            Ret[i] = Acm_AxAddPTData(m_Axishand[i], move_rel_pulse, time);                                                                // 添加第一笔 PT 数据
            time += delta_time;
        }
        time = 0; //reset the time
    }
}

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

        param.t0 = 0;
        param.tf = 10;
        param.samples_num = 500; //MUST BE SMALLER THAN 512

        Vec6d init_joint_pos, target_joint_pos;

        init_joint_pos << 0, 0, 0, 0, 0, 0;
        target_joint_pos << 0.2, 0.3, 1, 1, 0.2, 0.2;
        
        // for (int i = 0; i < m_ulAxisCount; i++)
        // {
        //     Acm_AxGetActualPosition(m_Axishand[i], &(init_pos_vec[i]));
        //     auto temp = inc2rad(init_pos_vec[i], i + 6); // NOTE:: TEST THE 2ND ROBOT at first.
        //     init_joint_pos(i) = temp;                    // assign the value
        // }

        std::cout << "rad of 6 joint pos: \n"
                  << init_joint_pos.transpose();
        std::cout << "Preview the planning motion?\n";

        std::cin >> temp_str;

        Eigen::Vector3d trans(0, 0, -50);
        //bool cont_flag = 0;

        //robo2.line_traj_planning(traj_planning_result_cont, init_joint_pos, trans, param);
        robo2.joint_space_planning(traj_planning_result_cont,init_joint_pos,target_joint_pos,param);
        preview_PTmove(mypub, traj_planning_result_cont, param);
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

bool set_axis_properties(const HAND *axis_hand, const AxisProperties &axisprop)
{
    U32 Ret;
    DOUBLE temp_FT;

    Ret = Acm_SetF64Property(*axis_hand, CFG_AxMaxAcc, axisprop.AxMaxAcc);
    Ret = Acm_GetF64Property(*axis_hand, CFG_AxMaxAcc, &temp_FT);
    std::cout << "Max Acc of Axis 0 is set as:" << std::dec << temp_FT << std::endl;

    Ret = Acm_SetF64Property(*axis_hand, CFG_AxMaxDec, axisprop.AxMaxDec);
    Ret = Acm_GetF64Property(*axis_hand, CFG_AxMaxDec, &temp_FT);
    std::cout << "Max Dec of Axis 0 is set as:" << std::dec << temp_FT << std::endl;
    Ret = Acm_SetF64Property(*axis_hand, CFG_AxMaxVel, axisprop.AxMaxVel);
    Ret = Acm_GetF64Property(*axis_hand, CFG_AxMaxVel, &temp_FT);
    std::cout << "Max Vel of Axis 0 is set as:" << std::dec << temp_FT << std::endl;

    Ret = Acm_SetF64Property(*axis_hand, PAR_AxVelLow, axisprop.AxVelLow);
    Ret = Acm_SetF64Property(*axis_hand, PAR_AxVelHigh, axisprop.AxVelHigh);
    Ret = Acm_SetF64Property(*axis_hand, PAR_AxAcc, axisprop.AxAcc);
    Ret = Acm_SetF64Property(*axis_hand, PAR_AxDec, axisprop.AxDec);
    Ret = Acm_SetF64Property(*axis_hand, PAR_AxJerk, axisprop.AxJerk);

    return true;
}

void out_rel_pos(const std::vector<HAND> &axis_hand, const std::vector<F64> &init_pos)
{
    F64 new_pos;
    U16 state;
    std::string str_state;
    //auto iter = axis_hand.cbegin();

    while (continue_thr1 == true)
    {
        for (int i = 0; i < axis_hand.size(); i++)
        {
            Acm_AxGetActualPosition(axis_hand[i], &new_pos);
            Acm_AxGetState(axis_hand[i], &state);
            GetAxisStateStr(str_state, state);
            std::cout << "Rel_pos: " << new_pos - init_pos[i] << "  Axis_State: " << str_state << "\n";
        }
        std::cout << "------------------------\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return;
}

int ReadnSet_PTdata_fromtxt(const HAND &m_Axishand, const std::string &Filename, int amount)
{
    std::string str;
    std::ifstream data(Filename.c_str());
    double pos, time;
    double time_pre = 0; // Check the validity of the data.
    int Ret;
    int i = 0;

    data >> std::scientific;
    data >> std::setprecision(17);
    std::cout << std::setprecision(17) << std::scientific;

    if (amount > 512)
    {
        std::cout << "The size of PT data must not be larger than 512\n";
        amount = 512;
    }

    Ret = Acm_AxResetPTData(m_Axishand); // reset PT planning data. The size of buffer is 512.

    while (i < amount)
    {
        data >> time >> pos;
        std::cout << "PT TIME: " << time << " PT POS:" << pos << "\n"; // num2 is time, num1 is pos;
        time *= 1000;                                                  // convert to ms
        if (time - time_pre <= 2)                                      //less than 2 ms
        {
            std::cout << "check PT data file format! The i+1th time data must be higher than the ith\n"; // Important. Prevent the control card to stop immediately
            return -2;
        }
        Ret = Acm_AxAddPTData(m_Axishand, pos, time); // Add a set of PT data. The 2nd arg is relative pos (PPU); The 3rd arg is time in ms.
        if ((i == 0) && (time != 0))
        {
            std::cout << "check the PT data file format!\n";
            return -1;
        }
        i++;
        time_pre = time;
    }
    std::cout << std::setprecision(7) << std::defaultfloat;
    std::cout << i << "sets of PT data has been written!\n";
    data.close();
    return Ret;
}

void GetAxisStateStr(std::string &str, U16 state_num)
{
    switch (state_num) // SOME CASES OF STA ARE NOT INCLUDED
    {
    case STA_AX_BUSY:
        str = "STA_AX_BUSY";
        break;
    case STA_AX_CONTI_MOT:
        str = "STA_AX_CONTI_MOT";
        break;
    case STA_AX_DISABLE:
        str = "STA_AX_DISABLE";
        break;
    case STA_AX_ERROR_STOP:
        str = "STA_AX_ERROR_STOP";
        break;
    case STA_AX_READY:
        str = "STA_AX_READY";
        break;
    case STA_AX_STOPPING:
        str = "STA_AX_STOPPING";
        break;
    case STA_AX_WAIT_PTP:
        str = "STA_AX_WAIT_PTP";
        break;
    case STA_AX_PTP_MOT:
        str = "STA_AX_PTP_MOT";
        break;
    default:
        str = "???";
        break;
    }
    return;
}