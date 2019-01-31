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
    const int vr_start;
    const int vl_start;

    sVarsIndex(int mpc_steps)
        : x_start(0),
        y_start(x_start + mpc_steps),
        th_start(x_start + mpc_steps),
        vr_start(th_start + mpc_steps),
        vl_start(vr_start + mpc_steps - 1)
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
