#include "MPC.h"
#include "common.h"

DiffFGEval::DiffFGEval()
    : m_mpc_steps(20),
    m_wd(0.25),
    m_dt(0.1),
    m_ref_velocity(1.),
    m_w_velocity(1.),
    m_w_orientation(1.),
    m_w_vchange(1.),
    idx(m_mpc_steps)
{
}

void DiffFGEval::LoadCoeffs(const Eigen::VectorXd& coeffs)
{
    m_coeffs = coeffs;
}

// Load parameters of the MPC
void DiffFGEval::LoadParams(const std::map<std::string, double> &params)
{
    common::LoadParameter("mpc_dt", params, m_dt);
    common::LoadParameter("mpc_steps", params, m_mpc_steps);
    common::LoadParameter("mpc_velocity_weight", params, m_w_velocity);
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

    // The cost function is made based on the reference velocity
    // for the robot and the orientation at each timepoint of the MPC controller
    // The orientation is calculated using a fitted polynomial on the path points
    for (int i = 0; i < (m_mpc_steps - 1); ++i)
    {
        const CppAD::AD<double> x0 = vars[idx.x_start + i];
        const CppAD::AD<double> th0 = vars[idx.th_start + i];
        const CppAD::AD<double> v0 = vars[idx.v_start + i];

         // atan(f'(x)) at time t -> orientation
        const CppAD::AD<double> ref_th = CppAD::atan(common::PolyDerivativeEval(m_coeffs, x0));

        fg[0] += m_w_velocity * CppAD::pow(v0 - m_ref_velocity, 2);
        fg[0] += m_w_orientation * CppAD::pow(th0 - ref_th, 2);
    }

    // This should constrain the acceleration of the robot
    for (int i = 1; i < (m_mpc_steps - 1); ++i)
    {
        const CppAD::AD<double> v0 = vars[idx.v_start + i - 1];
        const CppAD::AD<double> v1 = vars[idx.v_start + i];

        fg[0] += m_w_vchange * CppAD::pow(v1 - v0, 2);
    }

    fg[1 + idx.x_start]  = vars[idx.x_start];
    fg[1 + idx.y_start]  = vars[idx.y_start];
    fg[1 + idx.th_start] = vars[idx.th_start];

    // Solve the speed and steering controls based on
    // the parameters determined at minimizing cost function
    for (int i = 0; i < (m_mpc_steps - 1); ++i)
    {
        // The state at time t
        const CppAD::AD<double> x0  = vars[idx.x_start  + i];
        const CppAD::AD<double> y0  = vars[idx.y_start  + i];
        const CppAD::AD<double> th0 = vars[idx.th_start + i];

        // Actuators at time t
        const CppAD::AD<double> v0 = vars[idx.v_start + i];
        const CppAD::AD<double> s0 = vars[idx.s_start + i];

        // The state at time t+1
        const CppAD::AD<double> x1  = vars[idx.x_start  + i + 1];
        const CppAD::AD<double> y1  = vars[idx.y_start  + i + 1];
        const CppAD::AD<double> th1 = vars[idx.th_start + i + 1];

        fg[2 + idx.x_start  + i] = x1 - (x0 + v0 * CppAD::cos(th0) * m_dt);
        fg[2 + idx.y_start  + i] = y1 - (y0 + v0 * CppAD::sin(th0) * m_dt);
        fg[2 + idx.th_start + i] = th1 - (th0 + s0 * m_dt);
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
    for (int i = 0; i < m_fg_eval.idx.v_start; ++i)
    {
        vars_lowerbound[i] = -1e10; // almost no limit
        vars_upperbound[i] = 1e10;  // almost no limit
    }

    // Limits for the velocity
    for (int i = m_fg_eval.idx.v_start; i < m_fg_eval.idx.s_start; ++i)
    {
        vars_lowerbound[i] = 0.;
        vars_upperbound[i] = m_max_velocity;
    }

    for (int i = m_fg_eval.idx.s_start; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -1e10;
        vars_upperbound[i] = 1e10;
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
    //  [velocity, steering]
    return { solution.x[m_fg_eval.idx.v_start], solution.x[m_fg_eval.idx.s_start] };
}
