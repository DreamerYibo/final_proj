#include <windows.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <valarray>
#include <iomanip>
#include <Eigen/Dense>
#include "EfortKine.h"
#include "efort_control.h"

#define _USE_MATH_DEFINES
#include <math.h>

bool continue_thr1 = true;

int main() // 这个程序没有软件急停，用急停开关！
{
    using std::cin;
    using std::cout;
    using std::endl;

    joint_6dofPublisher mypub;
    //const std::string robot_name = "robot1"; // specify which robot to control and to preview! CRITICAL
    std::string robot_type = "ER20";
    int index_offset = 0; // robot 1 index offset

    std::vector<Vec6d> traj_planning_result_cont[2];           // clear it before each use of EfortKine::line_traj_planning()
    std::vector<Vec6d> target_joint_pos_series;                // save a series of joint pos config read from the txt file.
    std::vector<std::vector<Vec6d>> executing_PT_data_sets[2]; // push generated traj into this;

    std::valarray<U32> Ret(6);
    RoboEntity Robo[2];

    std::string temp_str, r;

    Ret[0] = Acm_GetDevNum(Adv_PCI1203, 0, (&Robo[0].m_DevNum)); // Get M0 DEVNUM. USE THE SAME DEVICE TO CONTROL ROBOT1,2,3 RESPECTIVELY
    Ret[0] = Acm_GetDevNum(Adv_PCI1203, 8, (&Robo[1].m_DevNum)); // Get M0 DEVNUM. USE THE SAME DEVICE TO CONTROL ROBOT1,2,3 RESPECTIVELY
    std::cout << "The M0 device number is " << Robo[0].m_DevNum << std::endl;
    std::cout << "The M8 device number is " << Robo[1].m_DevNum << std::endl;

    //cin >> temp;
    Ret[0] = Acm_DevOpen(Robo[0].m_DevNum, &(Robo[0].m_Devhand)); // Open M0 and get its Handle
    Ret[0] = Acm_DevOpen(Robo[1].m_DevNum, &(Robo[1].m_Devhand)); // Open M0 and get its Handle

    Ret[0] = Acm_GetU32Property(Robo[0].m_Devhand, FT_DevAxesCount, &(Robo[0].m_ulAxisCount)); //Get total number of axies of M0
    Ret[0] = Acm_GetU32Property(Robo[1].m_Devhand, FT_DevAxesCount, &(Robo[1].m_ulAxisCount)); //Get total number of axies of M0
    cout << "Return value for getting the M0 device's axises " << std::hex << Ret[0] << endl;
    cout << "The number of axises is:" << Robo[0].m_ulAxisCount << "\nThe number of axises is: " << Robo[1].m_ulAxisCount << endl;

    EfortRobo roboK("ER20"); // build the robo kinematic model

    for (int i = 0; i < 6; i++)
    {
        Ret[i] = Acm_AxOpen(Robo[0].m_Devhand, i, &(Robo[0].m_Axishand[i])); //get axis's handle
        Ret[i] = Acm_AxOpen(Robo[1].m_Devhand, i, &(Robo[1].m_Axishand[i])); //get axis's handle
    }

    AxisProperties prop3; // note inc per deg! This case is only valid for incpdeg ~~ 50000
    prop3.AxMaxAcc = 300000;
    prop3.AxMaxDec = 600000;
    prop3.AxMaxVel = 50000;
    prop3.AxAcc = 60000;
    prop3.AxDec = 60000;
    prop3.AxJerk = 0;
    prop3.AxVelHigh = 80000;
    prop3.AxVelLow = 2000;

    for (int i = 0; i < 6; i++) // servo on
    {
        double coeff = abs(incpdeg[i + index_offset] / 50000.0);
        prop3.AxMaxAcc = prop3.AxMaxAcc * coeff;
        prop3.AxMaxDec *= coeff;
        prop3.AxMaxVel *= coeff;
        prop3.AxAcc *= coeff;
        prop3.AxDec *= coeff;
        prop3.AxVelHigh *= coeff;
        prop3.AxVelLow *= coeff;

        std::cout << "Axis" << i << ":\n";
        std::cout << prop3.AxMaxAcc << " " << prop3.AxMaxDec << " " << prop3.AxMaxVel << " " << prop3.AxAcc << " " << prop3.AxDec << " " << prop3.AxVelHigh << " \n";

        set_axis_properties(&(Robo[0].m_Axishand[i]), prop3);
        Acm_AxResetError(Robo[0].m_Axishand[i]);
        Ret[i] = Acm_AxSetSvOn(Robo[0].m_Axishand[i], 1);
    }

    index_offset = 6;           // ROBOT2 CONFIG
    for (int i = 0; i < 6; i++) // servo on
    {
        double coeff = abs(incpdeg[i + index_offset] / 50000.0);
        prop3.AxMaxAcc = prop3.AxMaxAcc * coeff;
        prop3.AxMaxDec *= coeff;
        prop3.AxMaxVel *= coeff;
        prop3.AxAcc *= coeff;
        prop3.AxDec *= coeff;
        prop3.AxVelHigh *= coeff;
        prop3.AxVelLow *= coeff;

        std::cout << "Axis" << i << ":\n";
        std::cout << prop3.AxMaxAcc << " " << prop3.AxMaxDec << " " << prop3.AxMaxVel << " " << prop3.AxAcc << " " << prop3.AxDec << " " << prop3.AxVelHigh << " \n";

        set_axis_properties(&(Robo[1].m_Axishand[i]), prop3);
        Acm_AxResetError(Robo[1].m_Axishand[i]);
        Ret[i] = Acm_AxSetSvOn(Robo[1].m_Axishand[i], 1);
    }

    cout << "Turning on the servo of axis123456. Ret = " << Ret[0] << " " << Ret[1] << "" << Ret[2]
         << "" << Ret[3] << "" << Ret[4] << "" << Ret[5] << endl;

    cout << "Next step? (Y/N)" << endl;
    cin >> temp_str;
    while (temp_str == "N" || temp_str == "n")
    {
        for (int i = 0; i < 6; i++) // servo on
        {
            Ret[i] = Acm_AxSetSvOn(Robo[0].m_Axishand[i], 1);
            Ret[i] = Acm_AxSetSvOn(Robo[1].m_Axishand[i], 1);
            cout << "Turning on the servo of axis123456. Ret = " << Ret[0] << " " << Ret[1] << "" << Ret[2]
                 << "" << Ret[3] << "" << Ret[4] << "" << Ret[5] << endl;
        }
    }

    std::vector<F64> init_pos_vec(12);
    std::vector<HAND> m_Axishand_vec(12);
    PlanParam param; // traj_planning
    //std::vector<PlanParam> execute_PT_param_set;

    for (int i = 0; i < 6; i++) // put into the container
    {
        m_Axishand_vec[i] = Robo[0].m_Axishand[i];
        m_Axishand_vec[i + 6] = Robo[1].m_Axishand[i];
    }

    Vec6d init_joint_pos[2], P_joint_pos[2];

    init_joint_pos[0] << -0.875945, -0.301237, -0.435248, 0.872947, 0.960829, -0.485312;
    init_joint_pos[1] << 0.76824, -0.205981, -0.578078, -0.946207, 1.03896, -0.0829001;

    for (int j = 0; j < 2; j++)
    {
        P_joint_pos[j] = init_joint_pos[j];
    }

    // std::cout << init_joint_pos[0];
    // std::cout << init_joint_pos[1];

    if (mypub.init())
    {
        param.tf = 3; // mean ang vel should be lower than 10 degree/s.
        param.t0 = 0;
        param.samples_num = 60; //MUST BE SMALLER THAN 512
        Eigen::Vector3d trans[2];

        //move 1
        trans[0] << 0, -3, -0.3;
        trans[1] << 0, 3, 0;

        roboK.line_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], trans[0], param);
        roboK.line_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], trans[1], param);

        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //move 2
        trans[0] << 0, -6, 0;
        trans[1] << 0, 6, 0;

        roboK.line_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], trans[0], param);
        roboK.line_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], trans[1], param);

        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //move 3
        trans[0] << 0, 0, 200;
        trans[1] << 0, 0, 200;

        roboK.line_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], trans[0], param);
        roboK.line_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], trans[1], param);

        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        //move 4
        param.tf = 7; // mean ang vel should be lower than 10 degree/s.
        param.t0 = 0;
        param.samples_num = 120; //MUST BE SMALLER THAN 512
        Eigen::Vector3d k_axis[2], o_rel[2];
        k_axis[0] << 1, 0, 0;
        k_axis[1] << 1, 0, 0;
        o_rel[0] << 0, -118, 0;
        o_rel[1] << 0, 118, 0;

        roboK.rotate_arc_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], o_rel[0], k_axis[0], pi / 4.0, param);
        roboK.rotate_arc_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], o_rel[1], k_axis[1], pi / 4.0, param);
        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //move 5
        param.tf = 7; // mean ang vel should be lower than 10 degree/s.
        param.t0 = 0;
        param.samples_num = 120; //MUST BE SMALLER THAN 512
        k_axis[0] << 1, 0, 0;
        k_axis[1] << 1, 0, 0;
        o_rel[0] << 0, -118*cos(pi / 4.0), -118*sin(pi / 4.0);
        o_rel[1] << 0, 118*cos(pi / 4.0), 118*sin(pi / 4.0);

        roboK.rotate_arc_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], o_rel[0], k_axis[0], -pi / 4.0, param);
        roboK.rotate_arc_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], o_rel[1], k_axis[1], -pi / 4.0, param);
        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //move 6
        trans[0] << 0, 0, -200;
        trans[1] << 0, 0, -200;

        roboK.line_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], trans[0], param);
        roboK.line_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], trans[1], param);

        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //move 7
        trans[0] << 0, 80, 50;
        trans[1] << 0, -80, 50;

        roboK.line_traj_planning(traj_planning_result_cont[0], init_joint_pos[0], trans[0], param);
        roboK.line_traj_planning(traj_planning_result_cont[1], init_joint_pos[1], trans[1], param);

        preview_PTmove(mypub, traj_planning_result_cont, param);

        for (int i = 0; i < 2; i++)
        {
            init_joint_pos[i] = traj_planning_result_cont[i].back(); // update the init_joint_pos
            executing_PT_data_sets[i].push_back(traj_planning_result_cont[i]);
            traj_planning_result_cont[i].clear(); // clear the last result
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //std::thread thr1(out_rel_pos, m_Axishand_vec, init_pos_vec);

    if (executing_PT_data_sets[0].size() != executing_PT_data_sets[1].size())
    {
        std::cout << "\nexecuting PT sets are not the same between the two robos!\n";
        return -231;
    }

    for (int i = 0; i < executing_PT_data_sets[0].size(); i++)
    {
        if (i == 0)
        {
            PlanParam param2;
            param2.tf = 15; // mean ang vel should be lower than 10 degree/s.
            param2.t0 = 0;
            param2.samples_num = 100; //MUST BE SMALLER THAN 512
            for (int j = 0; j < 2; j++)
            {
                init_joint_pos[j] = read_joint_pos(Robo[j].m_Axishand, "robot" + std::to_string(j + 1));
                std::cout << "the robo" << j + 1 << " joint pos is:\n"
                          << init_joint_pos[j].transpose() << "\n";
                roboK.joint_space_planning(traj_planning_result_cont[j], init_joint_pos[j], P_joint_pos[j], param2); //back to the known P joint pos
                //preview_PTmove(mypub, traj_planning_result_cont, param2, "real_time", robot_name);
                set_PTdata(param2, traj_planning_result_cont[j], Robo[j].m_Axishand, "robot" + std::to_string(j + 1));
            }
            preview_PTmove(mypub, traj_planning_result_cont, param2);

            cout << "\nStart PT move?" << endl;
            cin >> temp_str;
            if (temp_str == "N" || temp_str == "n")
            {
                // close the device
                close_6_axises(Robo[0].m_Axishand);
                close_6_axises(Robo[1].m_Axishand);
                Ret[0] = Acm_DevClose(&(Robo[0].m_Devhand));
                Ret[0] = Acm_DevClose(&(Robo[1].m_Devhand));
                cout << "The device is closed with return value: " << Ret[0] << endl;
                return -1;
            }

            Ret = Acm_AxStartAllPT(&(Robo[0].m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6
            Ret = Acm_AxStartAllPT(&(Robo[1].m_Axishand[0]), 0, 6);

            cout << "Type in e or E to stop moving" << endl;
            cin >> temp_str;
            if (temp_str == "E" || temp_str == "e")
            {
                for (int j = 0; j < 2; j++)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        Acm_AxStopDec(Robo[j].m_Axishand[i]); // stop move.
                    }
                }
                close_6_axises(Robo[0].m_Axishand);
                close_6_axises(Robo[1].m_Axishand);
                return -1;
            }

            std::cout << "BACK TO P:\n";
            std::cout << read_joint_pos(Robo[0].m_Axishand, "robot1").transpose();
            std::cout << read_joint_pos(Robo[0].m_Axishand, "robot2").transpose();
            traj_planning_result_cont[0].clear(); // clear the last result
            traj_planning_result_cont[1].clear();

            while (!Axises_all_ready(Robo[0].m_Axishand) || !Axises_all_ready(Robo[1].m_Axishand)) // prevent writing new PT data before the last move is finished!!!!
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }

        param.tf = 15; // mean ang vel should be lower than 10 degree/s.
        param.t0 = 0;
        param.samples_num = 150; //MUST BE SMALLER THAN 512
        for (int j = 0; j < 2; j++)
        {
            set_PTdata(param, executing_PT_data_sets[j][i], Robo[j].m_Axishand, "robot" + std::to_string(j + 1));
        }
        cout << "\nStart PT move?" << endl;
        cin >> temp_str;
        if (temp_str == "N" || temp_str == "n")
        {
            // close the device
            close_6_axises(Robo[0].m_Axishand);
            close_6_axises(Robo[1].m_Axishand);
            Ret[0] = Acm_DevClose(&(Robo[0].m_Devhand));
            Ret[0] = Acm_DevClose(&(Robo[1].m_Devhand));
            cout << "The device is closed with return value: " << Ret[0] << endl;
            return -1;
        }
        Ret = Acm_AxStartAllPT(&(Robo[0].m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6
        Ret = Acm_AxStartAllPT(&(Robo[1].m_Axishand[0]), 0, 6);
        // while (1)
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     if (Axises_all_ready(m_Axishand))
        //         break;
        // }
        cout << "Type in e or E to stop moving" << endl;
        cin >> temp_str;
        if (temp_str == "E" || temp_str == "e")
        {
            for (int j = 0; j < 2; j++)
            {
                for (int i = 0; i < 6; i++)
                {
                    Acm_AxStopDec(Robo[j].m_Axishand[i]); // stop move.
                }
            }
            close_6_axises(Robo[0].m_Axishand);
            close_6_axises(Robo[1].m_Axishand);
            return -1;
        }
        while (!Axises_all_ready(Robo[0].m_Axishand) || !Axises_all_ready(Robo[1].m_Axishand)) // prevent writing new PT data before the last move is finished!!!!
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(7000)); // wait for 5 seconds after axies are all ready
    }
    // // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait until the axis stops.

    // continue_thr1 = 0;
    // thr1.join();
    // close the device
    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            Acm_AxSetSvOn((Robo[j].m_Axishand[i]), 0); //SVOFF
            Acm_AxClose(&(Robo[j].m_Axishand[i]));     //close axises.
        }
        Ret[0] = Acm_DevClose(&(Robo[j].m_Devhand));
    }

    cout << "The device is closed with return value: " << Ret[0] << endl;

    return 0;
}
