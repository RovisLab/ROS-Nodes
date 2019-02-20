#include "MPC.h"
#include "common.h"

#define val(x) CppAD::Value(CppAD::Var2Par(x))

DiffFGEval::DiffFGEval()
    : m_mpc_steps(20),
    m_wd(0.25),
    m_dt(0.1),
    m_ref_velocity(1.),
    m_w_velocity(1.),
    m_w_orientation(1.),
    m_w_vchange(1.),
    m_w_collision(1.),
    idx(m_mpc_steps)
{
}

void DiffFGEval::LoadCoeffs(
    const std::vector<Eigen::VectorXd>& coeffs_vec,
    size_t n_vars_per_robot,
    size_t n_contraints_per_robot)
{
    m_coeffs = coeffs_vec;
    m_vars_per_robot = n_vars_per_robot;
    m_constraints_per_robot = n_contraints_per_robot;
}

// Load parameters of the MPC
void DiffFGEval::LoadParams(const std::map<std::string, double> &params)
{
    common::LoadParameter("mpc_dt", params, m_dt);
    common::LoadParameter("mpc_steps", params, m_mpc_steps);
    common::LoadParameter("mpc_velocity_weight", params, m_w_velocity);
    common::LoadParameter("mpc_orientation_weight", params, m_w_orientation);
    common::LoadParameter("mpc_velocity_change_weight", params, m_w_vchange);
    common::LoadParameter("mpc_collision_weight", params, m_w_collision);
    common::LoadParameter("wheel_distance", params, m_wd);
    common::LoadParameter("reference_velocity", params, m_ref_velocity);

    new (&idx) sVarsIndex(m_mpc_steps);
}

// MPC implementation (cost func & constraints)
// fg: function that evaluates the objective and constraints using the syntax
void DiffFGEval::operator()(ADvector& fg, const ADvector& vars)
{
    const size_t NUM_ROBOTS = m_coeffs.size();

    // Transform predicted coordinates for all robots in the global coordinate system
    ADvector gx(NUM_ROBOTS * m_mpc_steps);
    ADvector gy(NUM_ROBOTS * m_mpc_steps);
    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const size_t coord_start = r * m_mpc_steps;
        const size_t vars_start = r * m_vars_per_robot;

        const double sinthr = sin(m_robots->at(r).getOrientation());
        const double costhr = cos(m_robots->at(r).getOrientation());

        for (int i = 0; i < m_mpc_steps; ++i)
        {
            const CppAD::AD<double> x = vars[vars_start + idx.x_start + i];
            const CppAD::AD<double> y = vars[vars_start + idx.y_start + i];

            // rotation and translation
            gx[coord_start + i] = (x * costhr - y * sinthr) + m_robots->at(r).getOdom().pose.pose.position.x;
            gx[coord_start + i] = (x * sinthr + y * costhr) + m_robots->at(r).getOdom().pose.pose.position.y;
        }
    }

    fg[0] = 0.; // cost function
    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const size_t start_idx = r * m_vars_per_robot;
        // The cost function is made based on the reference velocity
        // for the robot and the orientation at each timepoint of the MPC controller
        // The orientation is calculated using a fitted polynomial on the path points
        for (int i = 0; i < (m_mpc_steps - 1); ++i)
        {
            const CppAD::AD<double> x0  = vars[start_idx + idx.x_start  + i];
            const CppAD::AD<double> y0  = vars[start_idx + idx.y_start  + i];
            const CppAD::AD<double> th0 = vars[start_idx + idx.th_start + i];
            const CppAD::AD<double> v0  = vars[start_idx + idx.v_start  + i];

            const CppAD::AD<double> y_ref = common::PolyEval(m_coeffs[r], x0);
            // atan(f'(x)) at time t -> orientation
            const CppAD::AD<double> ref_th = CppAD::atan(common::PolyDerivativeEval(m_coeffs[r], x0));

            fg[0] += m_w_velocity * CppAD::pow(v0 - m_ref_velocity, 2);
            fg[0] += m_w_orientation * CppAD::pow(th0 - ref_th, 2);
            fg[0] += 0.5 * CppAD::pow(y_ref - y0, 2);
        }

        // This should constrain the acceleration of the robot
        for (int i = 1; i < (m_mpc_steps - 1); ++i)
        {
            const CppAD::AD<double> v0 = vars[start_idx + idx.v_start + i - 1];
            const CppAD::AD<double> v1 = vars[start_idx + idx.v_start + i];
            const CppAD::AD<double> s0 = vars[start_idx + idx.s_start + i - 1];
            const CppAD::AD<double> s1 = vars[start_idx + idx.s_start + i];

            fg[0] += m_w_vchange * CppAD::pow((v1 - v0) / m_dt, 2);
            //fg[0] += 0.1 * CppAD::pow((s1 - s0) / m_dt, 2);
        }

        // The radius of the collision circle
        const CppAD::AD<double> cc_radius = m_robots->at(r).getCollisionCircleRadius();
        const size_t coord_start = r * m_mpc_steps;

        for (size_t k = 0U; k < NUM_ROBOTS; ++k)
        {
            // Do not compute colliding cost with itself
            if (k == r)
            {
                continue;
            }

            const size_t other_coord_start = k * m_mpc_steps;

            // Compute cost at the same time step?
            for (int i = 1; i < m_mpc_steps; ++i)
            {
                const CppAD::AD<double> dist =
                    CppAD::sqrt(CppAD::pow(gx[other_coord_start + i] - gx[coord_start + i], 2) +
                                CppAD::pow(gy[other_coord_start + i] - gy[coord_start + i], 2));

                fg[0] += CppAD::CondExpLt(dist, cc_radius, // dist < cc_radius
                     m_w_collision * CppAD::pow(cc_radius - dist, 2), // if true
                     CppAD::AD<double>(0.)); // if false
            }
        }

        // for (size_t k = 0U; k < NUM_ROBOTS; ++k)
        // {
        //     if (k == r)
        //     {
        //         continue;
        //     }
        //
        //     const int other_start_idx = k * m_vars_per_robot;
        //
        //     for (int i = 0; i < m_mpc_steps; ++i)
        //     {
        //         const CppAD::AD<double> xr = vars[start_idx + idx.x_start + i];
        //         const CppAD::AD<double> yr = vars[start_idx + idx.y_start + i];
        //         const CppAD::AD<double> thr = vars[start_idx + idx.th_start + i];
        //
        //         const CppAD::AD<double> alpha = m_robots->at(r).getOrientation() + thr;
        //         const CppAD::AD<double> cosa = CppAD::cos(alpha);
        //         const CppAD::AD<double> sina = CppAD::sin(alpha);
        //
        //         // Points with:
        //         //  x coordinate = robot's x coordinate
        //         //  y coordinate = robot's y coordinate + fixed offset
        //         const CppAD::AD<double> ypt1 = yr + 10.;
        //         const CppAD::AD<double> ypt2 = yr - 10.;
        //
        //         // Rotate and translate the points which forms the line
        //         const CppAD::AD<double> x1 = (xr * cosa - ypt1 * sina) +
        //                                     m_robots->at(r).getOdom().pose.pose.position.x;
        //         const CppAD::AD<double> y1 = (xr * sina + ypt1 * cosa) +
        //                                     m_robots->at(r).getOdom().pose.pose.position.y;
        //
        //         const CppAD::AD<double> x2 = (xr * cosa - ypt2 * sina) +
        //                                     m_robots->at(r).getOdom().pose.pose.position.x;
        //         const CppAD::AD<double> y2 = (xr * sina + ypt2 * cosa) +
        //                                     m_robots->at(r).getOdom().pose.pose.position.y;
        //
        //         const CppAD::AD<double> xk = vars[other_start_idx + idx.x_start + i] +
        //                                     m_robots->at(k).getOdom().pose.pose.position.x;
        //         const CppAD::AD<double> yk = vars[other_start_idx + idx.y_start + i] +
        //                                     m_robots->at(k).getOdom().pose.pose.position.y;
        //
        //         // Calculate the distance from the robot to the line
        //         const CppAD::AD<double> num = CppAD::abs((y2 - y1)*xk - (x2 - x1)*yk + x2*y1 - y2*x1);
        //         const CppAD::AD<double> den = CppAD::sqrt(CppAD::pow(y2 - y1, 2) + CppAD::pow(x2 - x1, 2));
        //
        //         fg[0] += num / den;
        //     }
        // }

        const size_t c_start_idx = r * m_constraints_per_robot;

        fg[c_start_idx + 1 + idx.x_start]  = vars[start_idx + idx.x_start];
        fg[c_start_idx + 1 + idx.y_start]  = vars[start_idx + idx.y_start];
        fg[c_start_idx + 1 + idx.th_start] = vars[start_idx + idx.th_start];

        // Solve the speed and steering controls based on
        // the parameters determined at minimizing cost function
        for (int i = 0; i < (m_mpc_steps - 1); ++i)
        {
            // The state at time t
            const CppAD::AD<double> x0  = vars[start_idx + idx.x_start  + i];
            const CppAD::AD<double> y0  = vars[start_idx + idx.y_start  + i];
            const CppAD::AD<double> th0 = vars[start_idx + idx.th_start + i];

            // Actuators at time t
            const CppAD::AD<double> v0 = vars[start_idx + idx.v_start + i];
            const CppAD::AD<double> s0 = vars[start_idx + idx.s_start + i];

            // The state at time t+1
            const CppAD::AD<double> x1  = vars[start_idx + idx.x_start  + i + 1];
            const CppAD::AD<double> y1  = vars[start_idx + idx.y_start  + i + 1];
            const CppAD::AD<double> th1 = vars[start_idx + idx.th_start + i + 1];

            fg[c_start_idx + 2 + idx.x_start  + i] = x1 - (x0 + v0 * CppAD::cos(th0) * m_dt);
            fg[c_start_idx + 2 + idx.y_start  + i] = y1 - (y0 + v0 * CppAD::sin(th0) * m_dt);
            fg[c_start_idx + 2 + idx.th_start + i] = th1 - (th0 + s0 * m_dt);
        }
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
std::vector<std::vector<double>> DiffMPC::Solve(
    std::vector<Eigen::VectorXd> states_vec,
    const std::vector<Eigen::VectorXd>& coeffs_vec,
    std::vector<RobotWrapper>& robots_vec)
{
    const size_t NUM_ROBOTS = states_vec.size();

    // Set the number of model variables (includes both states and inputs)
    // For example: If the state is a state_vec.size() element vector, the actuators is a 2
    // element vector (velocity left, velocity right) and there are m_mpc_steps timestemps.
    // The number of variables is:
    const int NUM_ACTUATORS = 2;
    const size_t n_vars_per_robot = (m_mpc_steps * states_vec[0].size()) + ((m_mpc_steps - 1) * NUM_ACTUATORS);
    const size_t n_vars = NUM_ROBOTS * n_vars_per_robot;

    // Set the number of constraints
    const size_t n_state_constraints = m_mpc_steps * states_vec[0].size();
    //const size_t n_dist_contraints = m_mpc_steps * (NUM_ROBOTS - 1U);

    const size_t n_constraints_per_robot = n_state_constraints;// + n_dist_contraints;
    const size_t n_constraints = NUM_ROBOTS * n_constraints_per_robot;

    // Initial value of the independent variables.
    // Should be 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0U; i < n_vars; ++i)
    {
        vars[i] = 0.;
    }

    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const size_t start_idx = r * n_vars_per_robot;
        sRobotState state(states_vec[r]);

        // Initial values of the state
        vars[start_idx + m_fg_eval.idx.x_start]  = state.x;
        vars[start_idx + m_fg_eval.idx.y_start]  = state.y;
        vars[start_idx + m_fg_eval.idx.th_start] = state.th;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const int state_start_idx = r * n_vars_per_robot;
        const int state_end_idx = state_start_idx + m_fg_eval.idx.v_start;

        for (int i = state_start_idx; i < state_end_idx; ++i)
        {
            vars_lowerbound[i] = -1e10; // almost no limit
            vars_upperbound[i] = 1e10;  // almost no limit
        }

        // Limits for the velocity
        const int vel_end_idx = state_start_idx + m_fg_eval.idx.s_start;
        for (int i = state_end_idx; i < vel_end_idx; ++i)
        {
            vars_lowerbound[i] = 0.;
            vars_upperbound[i] = m_max_velocity;
        }

        const int s_end_idx = state_start_idx + n_vars_per_robot;
        for (int i = vel_end_idx; i < s_end_idx; ++i)
        {
            vars_lowerbound[i] = -3.14;
            vars_upperbound[i] = 3.14;
        }
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

    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const size_t start_idx = r * n_constraints_per_robot;
        sRobotState state(states_vec[r]);

        constraints_lowerbound[start_idx + m_fg_eval.idx.x_start]  = state.x;
        constraints_lowerbound[start_idx + m_fg_eval.idx.y_start]  = state.y;
        constraints_lowerbound[start_idx + m_fg_eval.idx.th_start] = state.th;

        constraints_upperbound[start_idx + m_fg_eval.idx.x_start]  = state.x;
        constraints_upperbound[start_idx + m_fg_eval.idx.y_start]  = state.y;
        constraints_upperbound[start_idx + m_fg_eval.idx.th_start] = state.th;
    }

    // Load coefficients in the evaluator
    m_fg_eval.LoadCoeffs(coeffs_vec, n_vars_per_robot, n_constraints_per_robot);
    m_fg_eval.LoadRobots(robots_vec);

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

    std::vector<std::vector<double>> control_vec;
    control_vec.resize(NUM_ROBOTS);

    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const size_t start_idx = r * n_vars_per_robot;
        robots_vec[r].predicted_x.resize(m_mpc_steps);
        robots_vec[r].predicted_y.resize(m_mpc_steps);
        for (int i = 0; i < m_mpc_steps; ++i)
        {
            robots_vec[r].predicted_x[i] = solution.x[start_idx + m_fg_eval.idx.x_start + i];
            robots_vec[r].predicted_y[i] = solution.x[start_idx + m_fg_eval.idx.y_start + i];
        }
    }

    const bool solutionFound = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!solutionFound)
    {
        ROS_ERROR("Solution not found (%d)!", solution.status);
        for (auto& vec : control_vec)
        {
            // 2 actuators
            vec = { 0., 0. };
        }

        return control_vec;
    }

    // Control vector:
    //  [velocity, steering]
    for (size_t r = 0U; r < NUM_ROBOTS; ++r)
    {
        const size_t start_idx = r * n_vars_per_robot;
        control_vec[r] = { solution.x[start_idx + m_fg_eval.idx.v_start], solution.x[start_idx + m_fg_eval.idx.s_start] };
    }

    return control_vec;
}
