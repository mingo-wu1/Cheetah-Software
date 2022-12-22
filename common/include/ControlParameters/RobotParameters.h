/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters which are shared between all robot controllers
 *  Currently there are some controlParameters that are specific to the MIT controllers here,
 *  but these will be moved in the future
 */

#ifndef PROJECT_ROBOTPARAMETERS_H
#define PROJECT_ROBOTPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

/*!
 * ControlParameters shared between all robot controllers
 */
class RobotControlParameters : public ControlParameters {
 public:

  /*!
   * Construct RobotControlParameters
   */
  RobotControlParameters()
      : ControlParameters("robot-parameters"),
        INIT_PARAMETER(myValue),
        INIT_PARAMETER(control_mode),
        INIT_PARAMETER(testValue),
        INIT_PARAMETER(controller_dt),
        INIT_PARAMETER(stand_kp_cartesian),
        INIT_PARAMETER(stand_kd_cartesian),
        INIT_PARAMETER(kpCOM),
        INIT_PARAMETER(kdCOM),
        INIT_PARAMETER(kpBase),
        INIT_PARAMETER(kdBase),
        INIT_PARAMETER(cheater_mode),
        INIT_PARAMETER(imu_process_noise_position),
        INIT_PARAMETER(imu_process_noise_velocity),
        INIT_PARAMETER(foot_process_noise_position),
        INIT_PARAMETER(foot_sensor_noise_position),
        INIT_PARAMETER(foot_sensor_noise_velocity),
        INIT_PARAMETER(foot_height_sensor_noise),
        INIT_PARAMETER(use_rc),

        /// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
        INIT_PARAMETER(use_cubicspline_for_qpstand_test),
        /// Add End
        /// Add Begin by wuchunming, 2021-03-17, BZL parameters, no use 3D Graphic Window
        INIT_PARAMETER(use_gfx){}
        /// Add End

  DECLARE_PARAMETER(double, myValue)
  DECLARE_PARAMETER(double, control_mode)
  DECLARE_PARAMETER(double, testValue)
  DECLARE_PARAMETER(double, controller_dt)
  DECLARE_PARAMETER(Vec3<double>, stand_kp_cartesian)
  DECLARE_PARAMETER(Vec3<double>, stand_kd_cartesian)
  DECLARE_PARAMETER(Vec3<double>, kpCOM)
  DECLARE_PARAMETER(Vec3<double>, kdCOM)
  DECLARE_PARAMETER(Vec3<double>, kpBase)
  DECLARE_PARAMETER(Vec3<double>, kdBase)

  // state estimator
  DECLARE_PARAMETER(s64, cheater_mode)
  DECLARE_PARAMETER(double, imu_process_noise_position)
  DECLARE_PARAMETER(double, imu_process_noise_velocity)
  DECLARE_PARAMETER(double, foot_process_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_velocity)
  DECLARE_PARAMETER(double, foot_height_sensor_noise)

  DECLARE_PARAMETER(s64, use_rc);
  
  /// Add Begin by wuchunming, 2021-04-19, add Cubic Spline Algorithm
  DECLARE_PARAMETER(s64, use_cubicspline_for_qpstand_test);
  /// Add End
  /// Add Begin by wuchunming, 2021-03-17, BZL parameters, no use 3D Graphic Window
  DECLARE_PARAMETER(s64, use_gfx);
  /// Add End
};

#endif  // PROJECT_ROBOTPARAMETERS_H
