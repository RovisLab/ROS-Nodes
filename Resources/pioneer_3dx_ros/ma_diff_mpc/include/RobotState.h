#ifndef ROBOT_STATE_MPC_DIFF_H
#define ROBOT_STATE_MPC_DIFF_H

enum EStateIndex
{
    X_POSITION,
    Y_POSITION,
    ORIENTATION,
    STATE_NUM
};

struct sVarsIndex
{
    const int x_start;
    const int y_start;
    const int th_start;
    const int v_start;
    const int s_start;

    sVarsIndex(int mpc_steps)
        : x_start(0),
        y_start(x_start + mpc_steps),
        th_start(y_start + mpc_steps),
        v_start(th_start + mpc_steps),
        s_start(v_start + mpc_steps - 1)
    {
    }
};

struct sRobotState
{
    double& x;  ///< x position
    double& y;  ///< y position
    double& th; ///< orientation

    template <typename Vector_t>
    sRobotState(Vector_t& state_vec)
        : x(state_vec[X_POSITION]),
        y(state_vec[Y_POSITION]),
        th(state_vec[ORIENTATION])
    {
    }
};

#endif
