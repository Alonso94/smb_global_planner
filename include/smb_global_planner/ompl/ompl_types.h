/*
 * ompl_typed.h
 * @brief Header for useful transformation with ompl and eigen
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: Jan 23, 2019
 */

#pragma once

#include <Eigen/Core>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl {
namespace smb {

// Take care of the position only
typedef base::RealVectorStateSpace RStateSpace;
typedef base::SE2StateSpace SE2StateSpace;

inline Eigen::Vector3d omplRToEigen(const base::State *state) {
  const smb::RStateSpace::StateType *derived =
      static_cast<const smb::RStateSpace::StateType *>(state);
  Eigen::Vector3d eigen_state(derived->values[0], derived->values[1],
                              derived->values[2]);

  return eigen_state;
}

inline Eigen::Vector3d omplSE2ToEigen(const base::State *state) {
  const smb::SE2StateSpace::StateType *derived =
      static_cast<const smb::SE2StateSpace::StateType *>(state);
  Eigen::Vector3d eigen_state(derived->getX(), derived->getY(),
                              derived->getYaw());

  return eigen_state;
}

} // end namespace smb
} // end namespace ompl
