/*! @file BZLPositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  BZLPositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "Controllers/BZLPositionVelocityEstimator.h"

template <typename T>
BZLLinearKFPositionVelocityEstimator<T>::BZLLinearKFPositionVelocityEstimator() {}

/*!
 * Initialize the state estimator
 */
template <typename T>
void BZLLinearKFPositionVelocityEstimator<T>::setup() {

}

/*!
 * Run state estimator
 */
template <typename T>
void BZLLinearKFPositionVelocityEstimator<T>::run() {

}

template class BZLLinearKFPositionVelocityEstimator<float>;
template class BZLLinearKFPositionVelocityEstimator<double>;