/*!
 * @file    bzl_quadruped_cmd.h
 * @author  hanyuanqiang
 * @brief   BZL quadruped API command and status definitione
 *
 */
#ifndef BZL_QUADRUPED_SDK_BZL_QUADRUPED_CMD_H
#define BZL_QUADRUPED_SDK_BZL_QUADRUPED_CMD_H

#include <stdint.h>

namespace BZL_QUADRUPED_SDK
{
    enum class RC_mode : int32_t {
        LOCOMOTION = 11,                                    // 运动模式
        STAND = 12,                                         // 站立
        PRONE = 14                                          // 趴下
    };

    /*!
    *  Body command message
    */
    typedef struct
    {
        RC_mode mode;
        float   forwardSpeed;                               // 前后（x轴）运行速度（m/s）
        float   sideSpeed;                                  // 左右（y轴）运行速度（m/s）
        float   rotateSpeed;                                // 左右旋转速度（rad/s）
        float   bodyHeight;                                 // reserve
        float   footRaiseHeight;                            // reserve
        float   yaw;                                        // reserve
        float   pitch;                                      // reserve
        float   roll;                                       // reserve
    } BodyCmd;

    /*!
    *  Body state message
    */
    typedef struct
    {
        RC_mode mode;
        int32_t errorCode;
        float   forwardSpeed;                               // 前后（x轴）速度（m/s）
        float   sideSpeed;                                  // 左右（y轴）速度（m/s）
        float   rotateSpeed;                                // 左右旋转速度（rad/s）
        float   bodyHeight;                                 // 车身高度（z轴）变化（m）
        float   forwardPosition;                            // 车身前后（x轴）变化（m）
        float   sidePosition;                               // 车身左右（y轴）变化（m）
    } BodyState; 

    /*!
    *  Gamepad command message
    */
    typedef struct
    {
        int32_t leftBumper;                                 // 踏步
        int32_t rightBumper;                                // 禁能（软急停）
        int32_t leftTriggerButton;                          // 切换步态
        int32_t rightTriggerButton;
        int32_t back;                                       // 爬坡/爬楼梯模式
        int32_t start;
        int32_t a;                                          // 趴下
        int32_t b;                                          // 站立
        int32_t x;                                          // 姿态械
        int32_t y;                                          // 后空翻
        int32_t leftStickButton;
        int32_t rightStickButton;                           
        int32_t buttonUp;                                   // 速度增加档位
        int32_t buttonDown;                                 // 速度减小档位
        int32_t buttonLeft;
        int32_t buttonRight;
        float   leftTriggerAnalog;
        float   rightTriggerAnalog;
        float   leftStickAnalog[2];                         // 左推杆（行走方向控制）
        float   rightStickAnalog[2];                        // 右推杆（旋转）
    } GamepadCmd;

    /*!
    *  Joint command message
    */
    typedef struct
    {
        float   qDesAbad[4];
        float   qDesHip[4];
        float   qDesKnee[4];

        float   qdDesAbad[4];
        float   qdDesHip[4];
        float   qdDesKnee[4];

        float   kpAbad[4];
        float   kpHip[4];
        float   kpKnee[4];

        float   kdAbad[4];
        float   kdHip[4];
        float   kdKnee[4];

        float   tauAbad[4];
        float   tauHip[4];
        float   tauKnee[4];

        int32_t flags[4];
    } JointCmd;

    /*!
    *  Joint state message
    */
    typedef struct
    {
        float   qAbad[4];
        float   qHip[4];
        float   qKnee[4];

        float   qdAbad[4];
        float   qdHip[4];
        float   qdKnee[4];

        int32_t  flags[4];
        int32_t  driverStatus;
    } JointState;

    /*!
    *  IMU message
    */
    typedef struct
    {
        float   acc[3];                                     // (m/s2)
        float   gyro[3];                                    // angular velocity （rad/s)
        float   rpy[3];                                     // euler angle（rad)
        float   quat[4];                                    // quaternion, normalized, (w,x,y,z)
    } IMU;

    /*!
    *  Sensor state message
    */
    typedef struct
    {
        IMU imu;
        int16_t footForce[4];                               // reserve
        int32_t power;                                      // reserve
    } SensorState;

}

#endif