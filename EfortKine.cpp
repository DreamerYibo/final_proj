#include "EfortKine.h"
#include <iomanip>

EfortRobo::EfortRobo(const std::string type)
{
    x << 0, 0, 0, 0, 0, 0;
    if (type == "ER20")
    {
        L1 = 504;
        L2 = 780;
        L3 = 140;
        L4 = 760;
        L5 = 125;
        L8 = 170;
        std::cout << "ER20 links' parameter: \n"
                  << L1 << " " << L2 << " " << L3 << " " << L4 << " " << L5 << " " << L8 << "\n";
    }
    if (type == "ER10")
    {
        L1 = 422;
        L2 = 681;
        L3 = 174;
        L4 = 745;
        L5 = 117;
        L8 = 195;
        std::cout << "ER10 links' parameter: \n"
                  << L1 << " " << L2 << " " << L3 << " " << L4 << " " << L5 << " " << L8 << "\n";
    }
}

Eigen::Matrix<double, 4, 4> &EfortRobo::T0_3(double x1, double x2, double x3)
{
    T_temp << -sin(x2 + x3) * cos(x1), sin(x1), cos(x2 + x3) * cos(x1), -cos(x1) * (L3 * sin(x2 + x3) - L8 + L2 * sin(x2)),
        -sin(x2 + x3) * sin(x1), -cos(x1), cos(x2 + x3) * sin(x1), -sin(x1) * (L3 * sin(x2 + x3) - L8 + L2 * sin(x2)),
        cos(x2 + x3), 0, sin(x2 + x3), L1 + L3 * cos(x2 + x3) + L2 * cos(x2),
        0, 0, 0, 1;
    return T_temp;
}
Eigen::Matrix<double, 4, 4> &EfortRobo::T0_4()
{
    T_temp << sin(x(0)) * sin(x(3)) - sin(x(1) + x(2)) * cos(x(0)) * cos(x(3)), -cos(x(1) + x(2)) * cos(x(0)), cos(x(3)) * sin(x(0)) + sin(x(1) + x(2)) * cos(x(0)) * sin(x(3)), L4 * cos(x(1) + x(2)) * cos(x(0)) - cos(x(0)) * (L3 * sin(x(1) + x(2)) - L8 + L2 * sin(x(1))),
        -cos(x(0)) * sin(x(3)) - sin(x(1) + x(2)) * cos(x(3)) * sin(x(0)), -cos(x(1) + x(2)) * sin(x(0)), sin(x(1) + x(2)) * sin(x(0)) * sin(x(3)) - cos(x(0)) * cos(x(3)), L4 * cos(x(1) + x(2)) * sin(x(0)) - sin(x(0)) * (L3 * sin(x(1) + x(2)) - L8 + L2 * sin(x(1))),
        cos(x(1) + x(2)) * cos(x(3)), -sin(x(1) + x(2)), -cos(x(1) + x(2)) * sin(x(3)), L1 + L3 * cos(x(1) + x(2)) + L4 * sin(x(1) + x(2)) + L2 * cos(x(1)),
        0, 0, 0, 1;
    return T_temp;
}

Eigen::Vector3d EfortRobo::T0_4_pos(double x1, double x2, double x3)
{
    Eigen::Vector3d pos;
    pos << L4 * cos(x2 + x3) * cos(x1) - cos(x1) * (L3 * sin(x2 + x3) - L8 + L2 * sin(x2)),
        L4 * cos(x2 + x3) * sin(x1) - sin(x1) * (L3 * sin(x2 + x3) - L8 + L2 * sin(x2)),
        L1 + L3 * cos(x2 + x3) + L4 * sin(x2 + x3) + L2 * cos(x2);
    return pos;
}

Eigen::Matrix<double, 4, 4> EfortRobo::forward_kine(Vec6d theta)
{
    x = theta;

    T_temp << sin(x(5)) * (cos(x(3)) * sin(x(0)) + sin(x(3)) * (cos(x(0)) * cos(x(1)) * sin(x(2)) + cos(x(0)) * cos(x(2)) * sin(x(1)))) + cos(x(5)) * (cos(x(4)) * (sin(x(0)) * sin(x(3)) - cos(x(3)) * (cos(x(0)) * cos(x(1)) * sin(x(2)) + cos(x(0)) * cos(x(2)) * sin(x(1)))) + sin(x(4)) * (cos(x(0)) * sin(x(1)) * sin(x(2)) - cos(x(0)) * cos(x(1)) * cos(x(2)))), cos(x(5)) * (cos(x(3)) * sin(x(0)) + sin(x(3)) * (cos(x(0)) * cos(x(1)) * sin(x(2)) + cos(x(0)) * cos(x(2)) * sin(x(1)))) - sin(x(5)) * (cos(x(4)) * (sin(x(0)) * sin(x(3)) - cos(x(3)) * (cos(x(0)) * cos(x(1)) * sin(x(2)) + cos(x(0)) * cos(x(2)) * sin(x(1)))) + sin(x(4)) * (cos(x(0)) * sin(x(1)) * sin(x(2)) - cos(x(0)) * cos(x(1)) * cos(x(2)))), sin(x(4)) * (sin(x(0)) * sin(x(3)) - cos(x(3)) * (cos(x(0)) * cos(x(1)) * sin(x(2)) + cos(x(0)) * cos(x(2)) * sin(x(1)))) - cos(x(4)) * (cos(x(0)) * sin(x(1)) * sin(x(2)) - cos(x(0)) * cos(x(1)) * cos(x(2))), L8 * cos(x(0)) + L4 * cos(x(1) + x(2)) * cos(x(0)) - L2 * cos(x(0)) * sin(x(1)) + L5 * cos(x(1) + x(2)) * cos(x(0)) * cos(x(4)) - L3 * cos(x(0)) * cos(x(1)) * sin(x(2)) - L3 * cos(x(0)) * cos(x(2)) * sin(x(1)) + L5 * sin(x(0)) * sin(x(3)) * sin(x(4)) - L5 * cos(x(0)) * cos(x(1)) * cos(x(3)) * sin(x(2)) * sin(x(4)) - L5 * cos(x(0)) * cos(x(2)) * cos(x(3)) * sin(x(1)) * sin(x(4)),
        -sin(x(5)) * (cos(x(0)) * cos(x(3)) - sin(x(3)) * (cos(x(1)) * sin(x(0)) * sin(x(2)) + cos(x(2)) * sin(x(0)) * sin(x(1)))) - cos(x(5)) * (cos(x(4)) * (cos(x(0)) * sin(x(3)) + cos(x(3)) * (cos(x(1)) * sin(x(0)) * sin(x(2)) + cos(x(2)) * sin(x(0)) * sin(x(1)))) + sin(x(4)) * (cos(x(1)) * cos(x(2)) * sin(x(0)) - sin(x(0)) * sin(x(1)) * sin(x(2)))), sin(x(5)) * (cos(x(4)) * (cos(x(0)) * sin(x(3)) + cos(x(3)) * (cos(x(1)) * sin(x(0)) * sin(x(2)) + cos(x(2)) * sin(x(0)) * sin(x(1)))) + sin(x(4)) * (cos(x(1)) * cos(x(2)) * sin(x(0)) - sin(x(0)) * sin(x(1)) * sin(x(2)))) - cos(x(5)) * (cos(x(0)) * cos(x(3)) - sin(x(3)) * (cos(x(1)) * sin(x(0)) * sin(x(2)) + cos(x(2)) * sin(x(0)) * sin(x(1)))), cos(x(4)) * (cos(x(1)) * cos(x(2)) * sin(x(0)) - sin(x(0)) * sin(x(1)) * sin(x(2))) - sin(x(4)) * (cos(x(0)) * sin(x(3)) + cos(x(3)) * (cos(x(1)) * sin(x(0)) * sin(x(2)) + cos(x(2)) * sin(x(0)) * sin(x(1)))), L8 * sin(x(0)) + L4 * cos(x(1) + x(2)) * sin(x(0)) - L2 * sin(x(0)) * sin(x(1)) + L5 * cos(x(1) + x(2)) * cos(x(4)) * sin(x(0)) - L3 * cos(x(1)) * sin(x(0)) * sin(x(2)) - L3 * cos(x(2)) * sin(x(0)) * sin(x(1)) - L5 * cos(x(0)) * sin(x(3)) * sin(x(4)) - L5 * cos(x(1)) * cos(x(3)) * sin(x(0)) * sin(x(2)) * sin(x(4)) - L5 * cos(x(2)) * cos(x(3)) * sin(x(0)) * sin(x(1)) * sin(x(4)),
        -cos(x(5)) * (sin(x(1) + x(2)) * sin(x(4)) - cos(x(1) + x(2)) * cos(x(3)) * cos(x(4))) - cos(x(1) + x(2)) * sin(x(3)) * sin(x(5)), sin(x(5)) * (sin(x(1) + x(2)) * sin(x(4)) - cos(x(1) + x(2)) * cos(x(3)) * cos(x(4))) - cos(x(1) + x(2)) * cos(x(5)) * sin(x(3)), sin(x(1) + x(2)) * cos(x(4)) + cos(x(1) + x(2)) * cos(x(3)) * sin(x(4)), L1 + L3 * cos(x(1) + x(2)) + L4 * sin(x(1) + x(2)) + L2 * cos(x(1)) + (L5 * cos(x(1) + x(2)) * sin(x(3) + x(4))) / 2 + L5 * sin(x(1) + x(2)) * cos(x(4)) - (L5 * sin(x(3) - x(4)) * cos(x(1) + x(2))) / 2,
        0, 0, 0, 1;

    return T_temp;
}

std::vector<Vec6d> EfortRobo::inv_kine(Eigen::Matrix<double, 4, 4> &kn_T0_6, std::string mode, Vec6d target_joint_pos)
{
    double pi = M_PI;
    using Eigen::Vector3d;

    Vector3d p, n, o, a;
    p = kn_T0_6.block<3, 1>(0, 3);
    n = kn_T0_6.block<3, 1>(0, 0);
    o = kn_T0_6.block<3, 1>(0, 1);
    a = kn_T0_6.block<3, 1>(0, 2);

    std::vector<Vector3d> x123_guess(8);
    std::vector<Vector3d> x123_true(4);

    double A, B, C;
    A = p(0) - L5 * a(0);
    B = p(1) - L5 * a(1);
    C = p(2) - L5 * a(2);

    for (int i = 0; i < 4; i++)
    {
        x123_guess[i](0) = atan2(B, A);
        x123_guess[i + 4](0) = atan2(-B, -A);
    }

    for (int i = 0; i < 8; i += 4) // guess x3
    {
        double x1 = x123_guess[i](0);
        double D = pow((A - cos(x1) * L8), 2) + pow((B - sin(x1) * L8), 2) + pow((C - L1), 2) - pow(L2, 2) - pow(L3, 2) - pow(L4, 2);
        double phi1 = atan2(L3, L4);

        double asin_tmp = D / (2 * L2 * sqrt(L3 * L3 + L4 * L4));

        if (abs(asin_tmp) > 1)
        {
            x123_guess[i](2) = nan("");
            x123_guess[i + 1](2) = nan("");
            x123_guess[i + 2](2) = nan("");
            x123_guess[i + 3](2) = nan("");
            continue;
        }
        else
        {
            double x3_guess = asin(asin_tmp) - phi1;
            double x3_guess2 = pi - asin(asin_tmp) - phi1;
            x123_guess[i](2) = x3_guess;
            x123_guess[i + 1](2) = x3_guess;
            x123_guess[i + 2](2) = x3_guess2;
            x123_guess[i + 3](2) = x3_guess2;
        }
    }

    for (int i = 0; i < 8; i += 2) // guess x2
    {
        double x1 = x123_guess[i](0);
        double x3 = x123_guess[i](2);

        if (isnan(x3))
        {
            x123_guess[i](1) = nan("");
            x123_guess[i + 1](1) = nan("");
        }
        else
        {
            double A_ = L3 * cos(x3) + L4 * sin(x3) + L2;
            double B_ = -L3 * sin(x3) + L4 * cos(x3);
            double phi2 = atan2(A_, B_);

            double temp = (C - L1) / sqrt(A_ * A_ + B_ * B_);
            x123_guess[i](1) = asin(temp) - phi2;
            x123_guess[i + 1](1) = pi - asin(temp) - phi2;
        }
    }

    for (int i = 0; i < 8; i++) // get rid of some fake guesses.
    {
        double x2 = x123_guess[i](1);
        double x3 = x123_guess[i](2);

        if (isnan(x3))
        {
            continue;
        }
        else
        {
            double op = sign(L4 * cos(x2 + x3) - L3 * sin(x2 + x3) + L8 - L2 * sin(x2));
            if (x123_guess[i](0) != atan2(B * op, A * op)) // the guess is invalid
            {
                // make corresponding x2 and x3 nan.
                x123_guess[i](1) = nan("");
                x123_guess[i](2) = nan("");
            }
        }
    }

    auto iter = x123_guess.cbegin();
    int count = 0; //count the number of true x123 solutions.

    //std::cout << "\n x123_guess: \n";
    while (iter != x123_guess.cend())
    {
        // std::cout << (*iter).transpose() << "\n"; //debug

        if (!isnan((*iter)(2))) // x3 is not nan
        {
            Vector3d diff;
            Vector3d guess_T0_4_pos = T0_4_pos((*iter)(0), (*iter)(1), (*iter)(2));
            diff << (guess_T0_4_pos(0) - A), (guess_T0_4_pos(1) - B), (guess_T0_4_pos(2) - C);
            if ((diff.array().abs() > 1e-3).any())
            {
                //std::cout << "debug\n";
            }
            else // the guess is valid
            {
                x123_true[count] = (*iter);
                count++;
            }
        }
        iter++;
    }

    if (count < 4)
    {
        int n = 4 - count;
        for (int i = 0; i < n; i++)
        {
            x123_true.pop_back(); //delete the unnecessary elements.
        }
    }

    Eigen::Matrix3d R3_6;
    std::vector<Vec6d> x_sol(2 * x123_true.size()); //inverse solution of theta( rad. Joint pos)

    for (int i = 0; i < 2 * x123_true.size(); i += 2)
    {
        int ind = i / 2;
        x_sol[i](0) = x123_true[ind](0);
        x_sol[i](1) = x123_true[ind](1);
        x_sol[i](2) = x123_true[ind](2);

        x_sol[i + 1](0) = x123_true[ind](0);
        x_sol[i + 1](1) = x123_true[ind](1);
        x_sol[i + 1](2) = x123_true[ind](2);

        T0_3(x123_true[ind](0), x123_true[ind](1), x123_true[ind](2)); //change T_temp as T0_3

        R3_6 = T_temp.block<3, 3>(0, 0).inverse() * kn_T0_6.block<3, 3>(0, 0); // inv(R0_3) * kn_T0_6(1:3, 1:3);

        Vector3d n_, o_, a_;
        n_ = R3_6.block<3, 1>(0, 0);
        o_ = R3_6.block<3, 1>(0, 1);
        a_ = R3_6.block<3, 1>(0, 2);

        x_sol[i](4) = acos(a_(2));      // get x5
        x_sol[i + 1](4) = -acos(a_(2)); // get x5

        double op = 0;
        if (sin(x_sol[i](4)) > 0)
        {
            op = 1;
        }
        else if (sin(x_sol[i](4)) < 0)
        {
            op = -1;
        }
        else
        {
            std::cout << "x5 has singularity???\n"; // This segment needs more work. This may lead to some problems.
            // if (a_(2) == 1) // when x5 = 0
            // {

            // }
        }

        x_sol[i](3) = atan2(a_(1) * op, a_(0) * op);
        x_sol[i](5) = atan2(o_(2) * op, -n_(2) * op);
        op = -op;
        x_sol[i + 1](3) = atan2(a_(1) * op, a_(0) * op);
        x_sol[i + 1](5) = atan2(o_(2) * op, -n_(2) * op);
    }

    if (mode != "search")
    {
        return x_sol;
    }
    else
    {
        auto x_sol_temp = x_sol; // save the abs diff between x_sol and target_joint_pos
        auto iter = x_sol_temp.begin();
        std::vector<double> abs_sum(x_sol.size());
        int i1 = 0;
        while (iter != x_sol_temp.end())
        {
            *(iter) = *(iter)-target_joint_pos;
            *(iter) = iter->cwiseAbs();
            //std::cout << "\n debug: " << iter->transpose();
            for (int i = 0; i < 6; i++)
            {
                if ((*iter)(i) > pi) // case: jump between [-pi, pi]
                {
                    (*iter)(i) = abs((*iter)(i)-2 * pi); // case: jump between [-pi, pi]
                }
            };

            abs_sum[i1] = iter->sum();

            i1++;
            iter++;
        }

        auto dis = std::distance(abs_sum.cbegin(), std::min_element(abs_sum.cbegin(), abs_sum.cend())); // find the closest sol index

        std::vector<Vec6d> x_sol_closest;
        auto iter2 = x_sol.cbegin();
        auto iter3 = x_sol_temp.cbegin();
        std::advance(iter2, dis);
        std::advance(iter3, dis);

        if (((*iter3).array() > (3 * pi / 180.0)).any()) // joint pos jerk more than 3 degree
        {
            std::cout << "WARNING: THE LINE_TRAJ PLANNING FCN HAS A JERK!!!\n";
            std::cout << "target_joint_pos:\n"
                      << target_joint_pos.transpose() << "\n";
            std::cout << "chosen_joint_plan_pos:\n"
                      << (*iter2).transpose() << "\n";
            std::cout << "x_sol:\n";

            // debug
            std::cout << std::fixed << std::setprecision(7);
            for (int i = 0; i < x_sol.size(); i++)
            {
                std::cout << x_sol[i].transpose() << "\n";
            }
            std::cout << std::defaultfloat;
        }

        x_sol_closest.push_back(*iter2);

        return x_sol_closest;
    }
}

double EfortRobo::sign(double num)
{
    if (num > 0)
    {
        return 1;
    }
    else if (num < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void EfortRobo::line_traj_planning(std::vector<Vec6d> &result_cont, Vec6d &init_joint_pos, Eigen::Vector3d translation, PlanParam &param)
{
    Eigen::Matrix4d init_mat = EfortRobo::forward_kine(init_joint_pos);
    Eigen::Vector3d start_pos = init_mat.block<3, 1>(0, 3);
    Eigen::Vector3d end_pos = start_pos + translation;

    double t = param.t0;
    double t0 = param.t0;
    double tf = param.tf;
    double delta_t = (tf - t0) / (param.samples_num - 1.0);

    Eigen::Matrix4d plan_T0_6 = Eigen::Matrix4d::Zero();

    std::cout << init_mat.block<3, 3>(0, 0); //debug

    Vec6d plan_joint_pos = init_joint_pos;

    // std::cout << "\ndebug 3\n";
    plan_T0_6.block<3, 3>(0, 0) = init_mat.block<3, 3>(0, 0); // the orientation stays the same
    plan_T0_6.block<1, 4>(3, 0) << 0, 0, 0, 1;
    // std::cout << "\ndebug 4\n";

    for (int i = 0; i < param.samples_num; i++)
    {
        double tau = (t - t0) / (tf - t0);
        double lambda = 10 * pow(tau, 3) - 15 * pow(tau, 4) + 6 * pow(tau, 5); //Zero initial condition for v and acc.

        Eigen::Vector3d position_plan = start_pos + lambda * (end_pos - start_pos);
        plan_T0_6.block<3, 1>(0, 3) = position_plan;

        plan_joint_pos = (inv_kine(plan_T0_6, "search", plan_joint_pos))[0];

        // std::cout << "\ndebug 2\n";

        result_cont.push_back(plan_joint_pos);

        t += delta_t;
    }
}

void EfortRobo::rotate_traj_planning(std::vector<Vec6d> &result_cont, Vec6d &init_joint_pos, Eigen::Vector3d k_axis, double alpha, PlanParam &param)
{
    Eigen::Matrix4d init_mat = EfortRobo::forward_kine(init_joint_pos);
    Eigen::Vector3d start_pos = init_mat.block<3, 1>(0, 3);
    Eigen::Vector3d end_pos = start_pos; //position would not be changed

    double t = param.t0;
    double t0 = param.t0;
    double tf = param.tf;
    double delta_t = (tf - t0) / (param.samples_num - 1.0);

    Eigen::Matrix4d plan_T0_6 = Eigen::Matrix4d::Zero();

    //std::cout << init_mat.block<3, 3>(0, 0); //debug

    Vec6d plan_joint_pos = init_joint_pos;

    // std::cout << "\ndebug 3\n";
    plan_T0_6.block<3, 1>(0, 3) = init_mat.block<3, 1>(0, 3); // the pos stays the same
    plan_T0_6.block<1, 4>(3, 0) << 0, 0, 0, 1;
    // std::cout << "\ndebug 4\n";

    Eigen::Matrix<double, 3, 3> pose_plan, pose_final;
    Eigen::Matrix<double, 3, 3> pose_init = init_mat.block<3, 3>(0, 0);
    pose_final = Rot_K(k_axis, alpha) * pose_init;
    plan_joint_pos = (inv_kine(plan_T0_6, "search", plan_joint_pos))[0];

    auto joint_mean_angular_vel = (plan_joint_pos - init_joint_pos).cwiseAbs() / (tf - t0);
    if ((joint_mean_angular_vel.array() > Max_ang_vel).any()) // angular vel should not be greater than 8 degree/s
    {
        std::cout << "22222222222joint_space_planning's tf is too low and the angular vel is too great!\n";
        double max_mean_vel = joint_mean_angular_vel.maxCoeff();
        tf = floor(tf * max_mean_vel / Max_ang_vel) + 1.0;
        delta_t = (tf - t0) / (param.samples_num - 1.0);
    }

    for (int i = 0; i < param.samples_num; i++)
    {
        double tau = (t - t0) / (tf - t0);
        double lambda = 10 * pow(tau, 3) - 15 * pow(tau, 4) + 6 * pow(tau, 5); //Zero initial condition for v and acc.
        //Eigen::Vector3d position_plan = start_pos + lambda * (end_pos - start_pos);
        pose_plan = Rot_K(k_axis, alpha * lambda) * pose_init;
        plan_T0_6.block<3, 3>(0, 0) = pose_plan;
        plan_joint_pos = (inv_kine(plan_T0_6, "search", plan_joint_pos))[0];

        // std::cout << "\ndebug 2\n";
        if (i > 0)
        {
            Vec6d temp_diff = (plan_joint_pos - result_cont.back()).cwiseAbs();
            if ((temp_diff.array() > 0.5).any())
            {
                std::cout << "\nWarining:rotate_traj_planning!!!\n"
                          << temp_diff.transpose() << "\n";
            }
        }
        else
        {
            Vec6d temp_diff = (plan_joint_pos - init_joint_pos).cwiseAbs();
            if ((temp_diff.array() > 0.5).any())
            {
                std::cout << "\nWarining:rotate_traj_planning!!!\n"
                          << temp_diff.transpose() << "\n";
            }
        }
        result_cont.push_back(plan_joint_pos);
        t += delta_t;
    }
}

void EfortRobo::joint_space_planning(std::vector<Vec6d> &result_cont, Vec6d &init_joint_pos, Vec6d &target_joint_pos, PlanParam &param)
{
    double t = param.t0;
    double t0 = param.t0;
    double tf = param.tf;
    double delta_t = (tf - t0) / (param.samples_num - 1.0);
    auto delta_joint_pos = target_joint_pos - init_joint_pos;

    auto joint_mean_angular_vel = (target_joint_pos - init_joint_pos).cwiseAbs() / (tf - t0);

    if ((joint_mean_angular_vel.array() > Max_ang_vel).any()) // angular vel should not be greater than 8 degree/s
    {
        std::cout << "joint_space_planning's tf is too low and the angular vel is too great!\n";
        double max_mean_vel = joint_mean_angular_vel.maxCoeff();
        tf = floor(tf * max_mean_vel / Max_ang_vel) * 1.2 + 1.0;
        param.tf = tf; // update tf
        std::cout << "\nnew tf: " << tf << "\n";
        delta_t = (tf - t0) / (param.samples_num - 1.0);
    }

    for (int i = 0; i < param.samples_num; i++)
    {
        double tau = (t - t0) / (tf - t0);
        double lambda = 10 * pow(tau, 3) - 15 * pow(tau, 4) + 6 * pow(tau, 5); //Zero initial condition for v and acc.
        auto joint_pos_plan = init_joint_pos + lambda * delta_joint_pos;
        result_cont.push_back(joint_pos_plan);
        t += delta_t;
    }
}

void param_normalize(PlanParam &param)
{
}

std::vector<double> linspace(double start, double end, unsigned int amount_of_points)
{
    if (amount_of_points < 2)
    {
        std::cout << "linspace must have at least 2 points!\n";
        exit(1);
    }

    double delta = (end - start) / ((double)amount_of_points - 1.0);
    std::vector<double> series(amount_of_points);

    auto iter = series.begin();
    double temp = start;

    while (iter != series.end())
    {
        (*iter) = temp;
        temp += delta;
        iter++;
    }
    return series;
}

Eigen::Matrix<double, 3, 3> Rot_K(Eigen::Vector3d K, double q)
{
    double cq = cos(q);
    double sq = sin(q);
    double vq = 1 - cos(q);

    double k_det = sqrt(K(0) * K(0) + K(1) * K(1) + K(2) * K(2));
    Eigen::Vector3d k_unit = K / k_det; // unify

    double kx = k_unit(0);
    double ky = k_unit(1);
    double kz = k_unit(2);

    Eigen::Matrix<double, 3, 3> R;

    R << kx * kx * vq + cq, ky * kx * vq - kz * sq, kz * kx * vq + ky * sq,
        kx * ky * vq + kz * sq, ky * ky * vq + cq, kz * ky * vq - kx * sq,
        kx * kz * vq - ky * sq, ky * kz * vq + kx * sq, kz * kz * vq + cq;

    return R;
}
