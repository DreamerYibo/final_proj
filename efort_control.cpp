#include "efort_control.h"

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
Vec6d read_joint_pos(const HAND *axis_hand_first, std::string robot_name)
{
    int indexoffset = 0; // used to call incpdeg function
    Vec6d joint_pos;

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
        Acm_AxGetActualPosition(axis_hand_first[i], &joint_pos(i));
        joint_pos(i) = inc2rad(joint_pos(i), i + indexoffset);
    }
    return joint_pos;
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

void Read_target_jointpos_sets(std::vector<Vec6d> &target_jointpos_cont, const std::string &Filename)
{
    std::string str;
    std::ifstream data(Filename.c_str());
    Vec6d joint_pos;

    data >> std::scientific;
    data >> std::setprecision(17);
    std::cout << std::setprecision(17) << std::scientific;

    while (!data.eof())
    {
        data >> joint_pos(0) >> joint_pos(1) >> joint_pos(2) >> joint_pos(3) >> joint_pos(4) >> joint_pos(5);
        target_jointpos_cont.push_back(joint_pos);
    }

    std::cout << std::setprecision(7) << std::defaultfloat;
    std::cout << target_jointpos_cont.size() << "sets of target joint data has been read!\n";
    data.close();
}

void preview_PTmove(joint_6dofPublisher &mypub, std::vector<Vec6d> &traj_planning_result_cont, PlanParam &param, std::string mode, std::string robot_name)
{
    Eigen::MatrixXd joint_input(6, 1);
    double publish_sleep_time = 10;
    int ind = 1; // robot 1 as default

    if (mode == "real_time")
    {
        publish_sleep_time = (param.tf - param.t0) / (param.samples_num - 1.0) * 1000;
    }
    else if (mode == "quick")
    {
        publish_sleep_time = 5;
    }
    if (robot_name == "robot1")
    {
        ind = 1;
    }
    else if (robot_name == "robot2")
    {
        ind = 2;
    }
    else if (robot_name == "robot3")
    {
        ind = 3;
    }
    else
    {
        std::cout << "Wrong Robot_name!\n";
    }
    for (int i = 0; i < traj_planning_result_cont.size(); i++)
    {
        joint_input = traj_planning_result_cont[i];
        mypub.publish(joint_input, ind);
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
            double move_rel_pulse; // relative amount of pulses need to move.
            double tau;
            tau = time / (param.tf - param.t0);

            // if ((tau > 0.04) || (tau < 0.97)) // Only write PT data in these cases.
            // {
            //     move_rel_pulse = (traj_planning_result_cont[j](i) - traj_planning_result_cont[0](i)) * 180.0 / pi * incpdeg[i + indexoffset]; // robo2. NOTE
            //     Ret[i] = Acm_AxAddPTData(m_Axishand[i], move_rel_pulse, time);                                                                // 添加第一笔 PT 数据
            // }
            // else if (tau == 0)
            // {
            //     Ret[i] = Acm_AxAddPTData(m_Axishand[i], 0, time);
            // }
            // else if (tau == 1)
            // {
            //     move_rel_pulse = (traj_planning_result_cont[j](i) - traj_planning_result_cont[0](i)) * 180.0 / pi * incpdeg[i + indexoffset]; // robo2. NOTE
            //     Ret[i] = Acm_AxAddPTData(m_Axishand[i], 0, time);
            // }

            move_rel_pulse = (traj_planning_result_cont[j](i) - traj_planning_result_cont[0](i)) * 180.0 / pi * incpdeg[i + indexoffset]; // robo2. NOTE
            if (j > 0)
            {
                if (abs(traj_planning_result_cont[j](i) - traj_planning_result_cont[j - 1](i)) > 1e-8) // IMPORTANT!!!!! SOMETIMES ONLY ONE AXIS MOVE! Do not add the same move rel value all the time!
                    Ret[i] = Acm_AxAddPTData(m_Axishand[i], move_rel_pulse, time);                     // 添加第一笔 PT 数据
            }
            else
            {
                Ret[i] = Acm_AxAddPTData(m_Axishand[i], move_rel_pulse, time); // 添加第一笔 PT 数据
            }
            time += delta_time;
        }
        time = 0; //reset the time
    }
}

bool close_6_axises(HAND *axis_hand)
{
    for (int i = 0; i < 6; i++)
    {
        Acm_AxSetSvOn((axis_hand[i]), 0); //SVOFF
        Acm_AxClose(&(axis_hand[i]));     // close axises.
    }
    return 1;
}

bool Axises_all_ready(HAND *axis_hand)
{
    bool result = 0;
    U16 state;
    std::string str_state;
    bool temp[6];
    //auto iter = axis_hand.cbegin();

    for (int i = 0; i < 6; i++)
    {
        Acm_AxGetState(axis_hand[i], &state);
        temp[i] = (state == STA_AX_READY);
    }
    result = (temp[0] && temp[1] && temp[2] && temp[3] && temp[4] && temp[5]);
    return result;
}