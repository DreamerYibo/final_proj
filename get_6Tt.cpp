// #include <basetsd.h>
// #include <windef.h>
// #include <winnt.h>
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

int main()
{
    using std::cin;
    using std::cout;
    using std::endl;

    std::ofstream file_out;
    joint_6dofPublisher mypub;
    const std::string robot_name = "robot3"; // specify which robot to control and to preview! CRITICAL
    std::string robot_type = "ER10";
    int index_offset = 0; // robot 1 index offset
    Eigen::MatrixXd joint_input(6, 1);
    joint_input << 1, 2, 3, 4, 5, 6;

    file_out.open(robot_name + "_get_6Tt.txt", std::ios_base::app);
    if (!file_out.is_open())
    {
        std::cout << robot_name + "_get_6Tt.txt"
                  << " failed!\n";
        return 23109;
    }
    else
    {
        auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        file_out << "\n-----------------------------" << ctime(&timenow) << "-----------------------------\n";
        file_out << std::scientific << std::setprecision(16);
    }

    std::vector<Vec6d> traj_planning_result_cont;           // clear it before each use of EfortKine::line_traj_planning()
    std::vector<std::vector<Vec6d>> executing_PT_data_sets; // push generated traj into this;

    std::valarray<U32> Ret(6);
    U32 m_ulAxisCount;
    HAND m_Devhand;
    U32 m_DevNum;
    HAND m_Axishand[6];

    std::string temp_str, r;

    Ret[0] = Acm_GetDevNum(Adv_PCI1203, 0, (&m_DevNum)); // Get M0 DEVNUM. USE THE SAME DEVICE TO CONTROL ROBOT1,2,3 RESPECTIVELY
    std::cout << "The M0 device number is " << m_DevNum << std::endl;

    //cin >> temp;
    Ret[0] = Acm_DevOpen(m_DevNum, &(m_Devhand));                              // Open M0 and get its Handle
    Ret[0] = Acm_GetU32Property(m_Devhand, FT_DevAxesCount, &(m_ulAxisCount)); //Get total number of axies of M0
    cout << "Return value for getting the M0 device's axises " << std::hex << Ret[0] << endl;
    cout << "The number of axises is:" << m_ulAxisCount << endl;

    if (robot_name == "robot1")
    {
        index_offset = 0;
        robot_type = "ER20";
    }
    else if (robot_name == "robot2")
    {
        index_offset = 6;
        robot_type = "ER20";
    }
    else if (robot_name == "robot3")
    {
        index_offset = 12;
        robot_type = "ER10";
    }
    else
    {
        std::cout << "Wrong Robot_name!\n";
        return 2213;
    }

    EfortRobo robo(robot_type); // build the robo kinematic model

    for (int i = 0; i < m_ulAxisCount; i++)
    {
        Ret[i] = Acm_AxOpen(m_Devhand, i, &(m_Axishand[i])); //get axis's handle
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

    for (int i = 0; i < m_ulAxisCount; i++) // servo on
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

        set_axis_properties(&(m_Axishand[i]), prop3);
        Acm_AxResetError(m_Axishand[i]);
        Ret[i] = Acm_AxSetSvOn(m_Axishand[i], 1);
    }

    cout << "Turning on the servo of axis123456. Ret = " << Ret[0] << " " << Ret[1] << "" << Ret[2]
         << "" << Ret[3] << "" << Ret[4] << "" << Ret[5] << endl;

    cout << "Next step? (Y/N)" << endl;
    cin >> temp_str;
    while (temp_str == "N" || temp_str == "n")
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
    std::vector<PlanParam> execute_PT_param_set;

    for (int i = 0; i < m_ulAxisCount; i++) // put into the container
    {
        m_Axishand_vec[i] = m_Axishand[i];
    }

    Vec6d init_joint_pos, target_joint_pos, P_joint_pos;

    if (robot_name == "robot1")
        init_joint_pos << -(pi * 5.0 / 7.0), 0.7, -0.2, 0.3, -0.3, 0.4; // init pos P
    else if (robot_name == "robot2")
        init_joint_pos << (pi * 4.0 / 7.0), 0.7, -0.3, -0.7, -0.3, -0.2; // init pos P
    else if (robot_name == "robot3")
        init_joint_pos << 0, 0.4, -0.1, 0.1, -0.6, 0.4; // init pos P
    else
    {
        std::cout << "Wrong Robot_name!\n";
        return 2215;
    }

    P_joint_pos = init_joint_pos; // init pos P
    if (mypub.init())
    {
        //std::vector<Vec6d> joint_pos_target_series;
        for (int i = 0; i < 4; i++) //rotate along x y and z respectively. ROTATE AROUND AXIS1  TO GET THE ORIGN'S X AND Y VALUE OF FRAME0!
        {
            if (i == 0 || i == 1)
            {
                // move 1
                param.tf = 14; // mean ang vel should be lower than  degree/s.
                param.t0 = 0;
                param.samples_num = 200; //MUST BE SMALLER THAN 512

                Eigen::Vector3d k_axis = (robo.forward_kine(P_joint_pos)).block<3, 1>(0, i); // along end's x or y axis
                robo.rotate_traj_planning(traj_planning_result_cont, init_joint_pos, k_axis, 0.4, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result

                // move 1_1 rotate again
                param.tf = 14; // mean ang vel should be lower than  degree/s.
                param.t0 = 0;
                param.samples_num = 200; //MUST BE SMALLER THAN 512

                //Eigen::Vector3d k_axis = (robo.forward_kine(P_joint_pos)).block<3, 1>(0, i); // along end's x or y axis
                robo.rotate_traj_planning(traj_planning_result_cont, init_joint_pos, k_axis, 0.3, param); //
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result

                // back to P
                param.tf = 14; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 200; //MUST BE SMALLER THAN 512
                robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, P_joint_pos, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);
                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                std::cout << "BACK TO P:\n";
                std::cout << traj_planning_result_cont.back().transpose();
                traj_planning_result_cont.clear(); // clear the last result
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // move 2
                param.tf = 14; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 200; //MUST BE SMALLER THAN 512

                //k_axis = (robo.forward_kine(P_joint_pos)).block<3, 1>(0, i); // along end's x or y axis
                robo.rotate_traj_planning(traj_planning_result_cont, init_joint_pos, k_axis, -0.4, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // move 2_1
                param.tf = 14; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 200; //MUST BE SMALLER THAN 512

                //k_axis = (robo.forward_kine(P_joint_pos)).block<3, 1>(0, i); // along end's x or y axis
                robo.rotate_traj_planning(traj_planning_result_cont, init_joint_pos, k_axis, -0.3, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // back to P
                param.tf = 14; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 200; //MUST BE SMALLER THAN 512
                robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, P_joint_pos, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);
                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                std::cout << "BACK TO P:\n";
                std::cout << traj_planning_result_cont.back().transpose();
                traj_planning_result_cont.clear(); // clear the last result
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            if (i == 2) //along z, only rotate joint 6!!!!!!
            {
                // move 1
                param.tf = 3; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 80; //MUST BE SMALLER THAN 512

                target_joint_pos = init_joint_pos;
                target_joint_pos(5) = target_joint_pos(5) + 0.3;

                robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, target_joint_pos, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result

                // move 2
                param.tf = 6; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 80; //MUST BE SMALLER THAN 512
                target_joint_pos = init_joint_pos;
                target_joint_pos(5) -= 0.8;

                robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, target_joint_pos, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result

                 // move 3 back
                param.tf = 6; // mean ang vel should be lower than 10 degree/s.
                param.t0 = 0;
                param.samples_num = 80; //MUST BE SMALLER THAN 512
                target_joint_pos = init_joint_pos;
                target_joint_pos(5) += 0.5;

                robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, target_joint_pos, param);
                preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                executing_PT_data_sets.push_back(traj_planning_result_cont);
                execute_PT_param_set.push_back(param);
                traj_planning_result_cont.clear(); // clear the last result
            }

            if (i == 3)
            {
                for (int j = 0; j < 4; j++)
                {
                    // move 1
                    param.tf = 3; // mean ang vel should be lower than 10 degree/s.
                    param.t0 = 0;
                    param.samples_num = 80; //MUST BE SMALLER THAN 512

                    target_joint_pos = init_joint_pos;
                    target_joint_pos(0) = target_joint_pos(0) - 0.3;

                    robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, target_joint_pos, param);
                    preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

                    init_joint_pos = traj_planning_result_cont.back(); // update the init_joint_pos
                    executing_PT_data_sets.push_back(traj_planning_result_cont);
                    execute_PT_param_set.push_back(param);
                    traj_planning_result_cont.clear(); // clear the last result
                }
            }
        }
    }

    std::thread thr1(out_rel_pos, m_Axishand_vec, init_pos_vec);
    for (int i = 0; i < executing_PT_data_sets.size(); i++)
    {
        if (i == 0)
        {
            init_joint_pos = read_joint_pos(m_Axishand, robot_name);
            std::cout << "the " << robot_name << " joint pos is:\n"
                      << init_joint_pos.transpose() << "\n";

            param.tf = 8; // mean ang vel should be lower than 10 degree/s.
            param.t0 = 0;
            param.samples_num = 80; //MUST BE SMALLER THAN 512
            robo.joint_space_planning(traj_planning_result_cont, init_joint_pos, P_joint_pos, param);
            preview_PTmove(mypub, traj_planning_result_cont, param, "quick", robot_name);

            set_PTdata(param, traj_planning_result_cont, m_Axishand, robot_name);
            cout << "\nStart PT move?" << endl;
            cin >> temp_str;
            if (temp_str == "N" || temp_str == "n")
            {
                // close the device
                close_6_axises(m_Axishand);
                Ret[0] = Acm_DevClose(&(m_Devhand));
                cout << "The device is closed with return value: " << Ret[0] << endl;
                return -1;
            }
            Ret = Acm_AxStartAllPT(&(m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6
            cout << "Type in e or E to stop moving" << endl;
            cin >> temp_str;
            if (temp_str == "E" || temp_str == "e")
            {
                for (int i = 0; i < 6; i++)
                {
                    Acm_AxStopDec(m_Axishand[i]); // stop move.
                    close_6_axises(m_Axishand);
                    return -1;
                }
            }
            std::cout << "BACK TO P:\n";
            std::cout << read_joint_pos(m_Axishand, robot_name).transpose();
            file_out << "P_pos_encoder:\n"
                     << read_joint_pos(m_Axishand, robot_name).transpose() << "\n";
            file_out << "P_pos_ideal:\n"
                     << traj_planning_result_cont.back().transpose() << "\n";
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait until the axis stops.
            // continue_thr1 = 0;
            // thr1.join();
            traj_planning_result_cont.clear(); // clear the last result
        }

        if (execute_PT_param_set.size() != executing_PT_data_sets.size())
        {
            std::cout << "\nexecute_PT_param_set.size()!=executing_PT_data_sets.size()\n";
            return 1111;
        }

        if (i < executing_PT_data_sets.size() - 3 - 4)
        {
            set_PTdata(execute_PT_param_set[i], executing_PT_data_sets[i], m_Axishand, robot_name);
            cout << "Start PT move?" << endl;

            cin >> temp_str;
            if (temp_str == "N" || temp_str == "n")
            {
                // close the device
                close_6_axises(m_Axishand);
                Ret[0] = Acm_DevClose(&(m_Devhand));
                cout << "The device is closed with return value: " << Ret[0] << endl;
                return 1;
            }

            // 执行 三个轴PT 运动
            // Ret = Acm_AxStartPT(m_Axishand[3], 0);
            Ret = Acm_AxStartAllPT(&(m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6
            cout << "Type in e or E to stop moving" << endl;
            cin >> temp_str;
            if (temp_str == "E" || temp_str == "e")
            {
                for (int i = 0; i < 6; i++)
                {
                    Acm_AxStopDec(m_Axishand[i]); // stop move.
                }
                close_6_axises(m_Axishand);
            }
            std::cout << read_joint_pos(m_Axishand, robot_name).transpose();
            file_out << i + 1 << " pos_encoder:\n"
                     << read_joint_pos(m_Axishand, robot_name).transpose() << "\n";
            file_out << i + 1 << "pos_ideal:\n"
                     << executing_PT_data_sets[i].back().transpose() << "\n";
        }
        else if (i < executing_PT_data_sets.size() - 4) // only execute movement of the joint 6!!!
        {
            set_PTdata(execute_PT_param_set[i], executing_PT_data_sets[i], m_Axishand, robot_name);
            cout << "Start PT move?" << endl;

            cin >> temp_str;
            if (temp_str == "N" || temp_str == "n")
            {
                // close the device
                close_6_axises(m_Axishand);
                Ret[0] = Acm_DevClose(&(m_Devhand));
                cout << "The device is closed with return value: " << Ret[0] << endl;
                return 1;
            }

            // 执行 三个轴PT 运动
            // Ret = Acm_AxStartPT(m_Axishand[3], 0);
            //Ret = Acm_AxStartAllPT(&(m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6

            Ret = Acm_AxStartPT(m_Axishand[5], 0);
            cout << "Type in e or E to stop moving" << endl;
            cin >> temp_str;
            if (temp_str == "E" || temp_str == "e")
            {
                for (int i = 0; i < 6; i++)
                {
                    Acm_AxStopDec(m_Axishand[i]); // stop move.
                }
                close_6_axises(m_Axishand);
            }
            file_out << i + 1 << " pos_encoder:\n"
                     << read_joint_pos(m_Axishand, robot_name).transpose() << "\n";
            file_out << i + 1 << "pos_ideal:\n"
                     << executing_PT_data_sets[i].back().transpose() << "\n";
        }
        else
        {
            set_PTdata(execute_PT_param_set[i], executing_PT_data_sets[i], m_Axishand, robot_name);
            cout << "Start PT move?" << endl;

            cin >> temp_str;
            if (temp_str == "N" || temp_str == "n")
            {
                // close the device
                close_6_axises(m_Axishand);
                Ret[0] = Acm_DevClose(&(m_Devhand));
                cout << "The device is closed with return value: " << Ret[0] << endl;
                return 1;
            }

            // 执行 三个轴PT 运动
            // Ret = Acm_AxStartPT(m_Axishand[3], 0);
            //Ret = Acm_AxStartAllPT(&(m_Axishand[0]), 0, 6); //start pt move for axis_hd1 - 6

            Ret = Acm_AxStartPT(m_Axishand[0], 0);
            cout << "Type in e or E to stop moving" << endl;
            cin >> temp_str;
            if (temp_str == "E" || temp_str == "e")
            {
                for (int i = 0; i < 6; i++)
                {
                    Acm_AxStopDec(m_Axishand[i]); // stop move.
                }
                close_6_axises(m_Axishand);
            }
            file_out << i + 1 << " pos_encoder:\n"
                     << read_joint_pos(m_Axishand, robot_name).transpose() << "\n";
            file_out << i + 1 << "pos_ideal:\n"
                     << executing_PT_data_sets[i].back().transpose() << "\n";
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait until the axis stops.
    }

    continue_thr1 = 0;
    thr1.join();
    // close the device
    for (int i = 0; i < 6; i++)
    {
        Acm_AxSetSvOn((m_Axishand[i]), 0); //SVOFF
        Acm_AxClose(&(m_Axishand[i]));     //close axises.
    }

    Ret[0] = Acm_DevClose(&(m_Devhand));
    cout << "The device is closed with return value: " << Ret[0] << endl;

    return 0;
}
