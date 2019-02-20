#ifndef DIFF_MPC_H
#define DIFF_MPC_H

#include <vector>
#include <map>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>
#include <ros/ros.h>

#include "RobotState.h"
#include "RobotWrapper.h"

class DiffFGEval
{
private:
    int m_mpc_steps;

    double m_wd;
    double m_dt;
    double m_ref_velocity;

    double m_w_velocity;
    double m_w_orientation;
    double m_w_vchange;
    double m_w_collision;

    std::vector<Eigen::VectorXd> m_coeffs;
    size_t m_vars_per_robot;
    size_t m_constraints_per_robot;

    const std::vector<RobotWrapper>* m_robots;

public:
    using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);

    sVarsIndex idx;

    // Constructor
    DiffFGEval();
    DiffFGEval(const DiffFGEval&) = default;
    DiffFGEval& operator=(const DiffFGEval&) = default;
    ~DiffFGEval() = default;

    void LoadCoeffs(
        const std::vector<Eigen::VectorXd>& coeffs_vec,
        size_t n_vars_per_robot,
        size_t n_constraints_per_robot);

    void LoadRobots(const std::vector<RobotWrapper>& robots_vec)
    {
        m_robots = &robots_vec;
    }

    void LoadParams(const std::map<std::string, double> &params);

    void operator()(ADvector& fg, const ADvector& vars);
};

class DiffMPC
{
    public:
        using Dvector = CPPAD_TESTVECTOR(double);

        DiffMPC();
        DiffMPC(const DiffMPC&) = default;
        DiffMPC& operator=(const DiffMPC&) = default;
        ~DiffMPC() = default;

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuations
        std::vector<std::vector<double>> Solve(
            std::vector<Eigen::VectorXd> states_vec,
            const std::vector<Eigen::VectorXd>& coeffs_vec,
            std::vector<RobotWrapper>& robots_vec);

        void LoadParams(const std::map<std::string, double>& params);

        std::vector<double> predicted_x;
        std::vector<double> predicted_y;

    private:
        int m_mpc_steps;
        double m_max_velocity;

        DiffFGEval m_fg_eval;
};

#endif /* MPC_H */
