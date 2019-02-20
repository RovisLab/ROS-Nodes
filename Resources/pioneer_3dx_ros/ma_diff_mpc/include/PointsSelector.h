#ifndef MA_MPC_DIFF_POINT_SELECTOR_H
#define MA_MPC_DIFF_POINT_SELECTOR_H

#include "common.h"

template <typename Path_t>
class PointsSelector
{
private:
    Path_t& m_path;

public:
    using value_type = std::pair<size_t, size_t>;

    PointsSelector(Path_t& path)
        : m_path(path)
    {
    }

    void reset()
    {
    }

    template <typename Pose_t>
    value_type select(const Pose_t& robotPos, double mpc_dist)
    {
        std::vector<double> distances(m_path.poses.size());
        for (size_t i = 0U; i < m_path.poses.size(); ++i)
        {
            distances[i] = common::Distance(m_path.poses[i].pose, robotPos);
        }

        const auto minDistIter = std::min_element(distances.begin(), distances.end());
        const auto first_idx = minDistIter - distances.begin();

        // Compute the last index on the path points
        size_t last_idx = 0U;
        for (size_t i = first_idx; i < m_path.poses.size(); ++i)
        {
            const double crt_dist = common::Distance(m_path.poses[i].pose, robotPos);
            if (crt_dist > mpc_dist)
            {
                last_idx = i;
                break;
            }
        }

        if (last_idx < first_idx)
        {
            last_idx = m_path.poses.size();
        }

        return { first_idx, last_idx };
    }
};

#endif
