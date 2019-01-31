/*
# Copyright (c) 2018 Elektrobit Automotive
# Developer: Mihai, Zaha (mihai.zaha@elektrobit.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include "MPC.h"
#include "common.h"

DiffFGEval::DiffFGEval()
    : m_mpc_steps(20),
    m_wd(0.25),
    m_dt(0.1),
    m_ref_velocity(1.),
    m_w_position(1.),
    m_w_orientation(1.),
    m_w_vchange(1.),
    idx(m_mpc_steps),
    m_goal_x(0.),
    m_goal_y(0.)
{
}

void DiffFGEval::LoadCoeffs(const Eigen::VectorXd& coeffs)
{
    m_coeffs = coeffs;
}

void DiffFGEval::SetGoal(double x, double y)
{
    m_goal_x = x;
    m_goal_y = y;
}

// Load parameters for constraints
void DiffFGEval::LoadParams(const std::map<std::string, double> &params)
{
    common::LoadParameter("mpc_dt", params, m_dt);
    common::LoadParameter("mpc_steps", params, m_mpc_steps);
    common::LoadParameter("mpc_position_weight", params, m_w_position);
    common::LoadParameter("mpc_orientation_weight", params, m_w_orientation);
    common::LoadParameter("mpc_velocity_change_weight", params, m_w_vchange);

    common::LoadParameter("wheel_distance", params, m_wd);
    common::LoadParameter("reference_velocity", params, m_ref_velocity);

    new (&idx) sVarsIndex(m_mpc_steps);
}

// MPC implementation (cost func & constraints)
// fg: function that evaluates the objective and constraints using the syntax
void DiffFGEval::operator()(ADvector& fg, const ADvector& vars)
{
    fg[0] = 0.;

    std::vector<double> x_ref;
    x_ref.resize(m_mpc_steps);

    const double sign = CppAD::Value(CppAD::Var2Par(CppAD::sign(m_goal_x - vars[idx.x_start])));
    const double step = sign * m_ref_velocity * m_dt;
    for (int i = 0; i < m_mpc_steps; ++i)
    {
        x_ref[i] = i * step;
    }

    for (int i = 0; i < m_mpc_steps; ++i)
    {
        const CppAD::AD<double> x0  = vars[idx.x_start  + i];
        const CppAD::AD<double> y0  = vars[idx.y_start  + i];
        const CppAD::AD<double> th0 = vars[idx.th_start + i];

        const CppAD::AD<double> y_ref = common::PolyEval(m_coeffs, x_ref[i]); // y=f(x) at time t
        const CppAD::AD<double> th_ref = CppAD::atan(common::PolyDerivativeEval(m_coeffs, x_ref[i])); // atan(f'(x)) at time t -> orientation

        fg[0] += m_w_position * CppAD::pow(x_ref[i] - x0, 2);
        fg[0] += m_w_position * CppAD::pow(y_ref - y0, 2);
        fg[0] += m_w_orientation * CppAD::pow(th_ref - th0, 2);
    }

    for (int i = 1; i < (m_mpc_steps - 1); ++i)
    {
        const CppAD::AD<double> vr0 = vars[idx.vr_start + i - 1];
        const CppAD::AD<double> vl0 = vars[idx.vl_start + i - 1];
        const CppAD::AD<double> vr1 = vars[idx.vr_start + i];
        const CppAD::AD<double> vl1 = vars[idx.vl_start + i];

        fg[0] += m_w_vchange * CppAD::pow(vr1 - vr0, 2);
        fg[0] += m_w_vchange * CppAD::pow(vl1 - vl0, 2);
    }

    fg[1 + idx.x_start]  = vars[idx.x_start];
    fg[1 + idx.y_start]  = vars[idx.y_start];
    fg[1 + idx.th_start] = vars[idx.th_start];

    for (int i = 0; i < (m_mpc_steps - 1); ++i)
    {
        // The state at time t
        const CppAD::AD<double> x0  = vars[idx.x_start  + i];
        const CppAD::AD<double> y0  = vars[idx.y_start  + i];
        const CppAD::AD<double> th0 = vars[idx.th_start + i];

        // Actuators at time t
        const CppAD::AD<double> vr0 = vars[idx.vr_start + i];
        const CppAD::AD<double> vl0 = vars[idx.vl_start + i];

        // The state at time t+1
        const CppAD::AD<double> x1  = vars[idx.x_start  + i + 1];
        const CppAD::AD<double> y1  = vars[idx.y_start  + i + 1];
        const CppAD::AD<double> th1 = vars[idx.th_start + i + 1];

        const CppAD::AD<double> alpha = m_dt * (vr0 - vl0) / m_wd;

        fg[2 + idx.x_start  + i] = x1 - (x0 + (vr0 + vl0) * 0.5 * CppAD::cos(th0) * m_dt);
        fg[2 + idx.y_start  + i] = y1 - (y0 + (vr0 + vl0) * 0.5 * CppAD::sin(th0) * m_dt);
        fg[2 + idx.th_start + i] = th1 - (th0 + alpha);
    }
}


DiffMPC::DiffMPC()
    : m_mpc_steps(20),
    m_max_velocity(1.0)
{
}

void DiffMPC::LoadParams(const std::map<std::string, double> &params)
{
    common::LoadParameter("mpc_steps", params, m_mpc_steps);
    common::LoadParameter("max_velocity", params, m_max_velocity);

    m_fg_eval.LoadParams(params);
}

void DiffMPC::SetGoal(double x, double y)
{
    m_fg_eval.SetGoal(x, y);
}

// state vector: [x, y, w]
std::vector<double> DiffMPC::Solve(Eigen::VectorXd state_vec, const Eigen::VectorXd& coeffs)
{
    sRobotState state(state_vec);

    // Set the number of model variables (includes both states and inputs)
    // For example: If the state is a state_vec.size() element vector, the actuators is a 2
    // element vector (velocity left, velocity right) and there are m_mpc_steps timestemps.
    // The number of variables is:
    const int NUM_ACTUATORS = 2;
    const size_t n_vars = (m_mpc_steps * state_vec.size()) + ((m_mpc_steps - 1) * NUM_ACTUATORS);

    // Set the number of constraints
    const size_t n_constraints = m_mpc_steps * state_vec.size();

    // Initial value of the independent variables.
    // Should be 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0U; i < n_vars; ++i)
    {
        vars[i] = 0.;
    }

    // Initial values of the state
    vars[m_fg_eval.idx.x_start]  = state.x;
    vars[m_fg_eval.idx.y_start]  = state.y;
    vars[m_fg_eval.idx.th_start] = state.th;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    for (int i = 0; i < m_fg_eval.idx.vr_start; ++i)
    {
        vars_lowerbound[i] = -1e10; // almost no limit
        vars_upperbound[i] = 1e10;  // almost no limit
    }

    // Limits for the velocity
    for (int i = m_fg_eval.idx.vr_start; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -m_max_velocity;
        vars_upperbound[i] = m_max_velocity;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0U; i < n_constraints; ++i)
    {
        constraints_lowerbound[i] = 0.;
        constraints_upperbound[i] = 0.;
    }

    constraints_lowerbound[m_fg_eval.idx.x_start]  = state.x;
    constraints_lowerbound[m_fg_eval.idx.y_start]  = state.y;
    constraints_lowerbound[m_fg_eval.idx.th_start] = state.th;

    constraints_upperbound[m_fg_eval.idx.x_start]  = state.x;
    constraints_upperbound[m_fg_eval.idx.y_start]  = state.y;
    constraints_upperbound[m_fg_eval.idx.th_start] = state.th;

    // Load coefficients in the evaluator
    m_fg_eval.LoadCoeffs(coeffs);

    std::string options;
    // print_level = 0 means no output
    options += "Integer print_level  0\n";
    // Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // Solver has a maximum time limit of 0.5 seconds.
    options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, DiffFGEval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, m_fg_eval, solution);

    const bool solutionFound = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!solutionFound)
    {
        ROS_ERROR("Solution not found!");
        return { 0., 0. };
    }

    // Predict path based on MPC output
    predicted_x.clear();
    predicted_y.clear();
    predicted_x.reserve(m_mpc_steps);
    predicted_y.reserve(m_mpc_steps);
    for (int i = 0; i < m_mpc_steps; ++i)
    {
        predicted_x.emplace_back(solution.x[m_fg_eval.idx.x_start + i]);
        predicted_y.emplace_back(solution.x[m_fg_eval.idx.y_start + i]);
    }

    // Control vector:
    //  [velocity right, velocity left]
    return { solution.x[m_fg_eval.idx.vr_start], solution.x[m_fg_eval.idx.vl_start] };
}
