# 1. SDK安装

## 1.1. 依赖安装

bzl-quadruped-sdk依赖lcm，建议使用v1.4.0版本：https://github.com/lcm-proj/lcm/releases/download/v1.4.0/lcm-1.4.0.zip。

如果是ubuntu18.04或更高版本，可直接使用apt源安装：

```shell
sudo apt-get install lcm
```

## 1.2. 安装说明

整个SDK通过deb安装包的方式提供，通过以下命令安装

```shell
sudo dpkg -i bzl-quadruped-sdk-x.x.x-xxx.deb
```

当用户在`/opt/bzl_quadruped_sdk`目录里面看到这份`README.md`时，说明用户已成功安装了bzl_quadruped_sdk，可通过以下命令查看sdk版本：

```shell
dpkg -l | grep -i bzl-quadruped-sdk
```



# 2. SDK说明

deb安装完成后将相应的头文件、动态库、示例工程安装到以下目录路径： 

```
├── opt
│   └── bzl_quadruped_sdk
│       ├── example
│       │   ├── body_cmd.cpp
│       │   ├── CMakeLists.txt
│       │   └── gamepad_cmd.cpp
│       └── README.md
└── usr
    └── local
        ├── include
        │   └── bzl_quadruped_sdk
        │       ├── bzl_quadruped_cmd.h
        │       └── bzl_quadruped.h
        └── lib
            ├── cmake
            │   └── bzl_quadruped_sdk
            │       ├── bzl_quadruped_sdk-config.cmake
            │       └── bzl_quadruped_sdk-config-release.cmake
            └── libbzl_quadruped_sdk.so
```

## 2.1. 头文件 

头文件安装到路径`/usr/local/include/bzl_quadruped_sdk`里面，使用过程中用户代码只需要包含头文件`bzl_quadruped.h`（API接口），该头文件自动包含`bzl_quadruped_cmd.h`（消息结构定义）。

```c++
// 包含sdk头文件方式
#include <bzl_quadruped_sdk/bzl_quadruped.h>
```

- **API接口定义**

  ```c++
  class BzlQuadruped
  {
  public:
      BzlQuadruped();
      ~BzlQuadruped();
      bool SendCmd(const BodyCmd& bodyCmd);
      bool SendCmd(const GamepadCmd& gamepadCmd);
      bool SendCmd(const JointCmd& jointCmd);
      bool ShutDown(void);                // 注意非紧急情况或者趴下状态下，不可调用该函数，调用该函数后，机器需要重启才能继续工作
      bool GetState(BodyState& bodyState);
      bool GetState(JointState& jointState);
      bool GetState(SensorState& sensorState);
  }; 
  ```

  二次开发所有的操作封装在了`BzlQuadruped`类里面，主要提供了整机控制、手柄控制、关节控制（预留）、整机状态 、关节状态、传感器状态6个接口功能。

- **整机控制消息结构定义（优先使用）**

  ```c++
  enum class RC_mode : int32_t {
      LOCOMOTION = 11,                                    // 运动模式
      STAND = 12,                                         // 站立
      PRONE = 14                                          // 趴下
  };
  
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
  ```

  只有当`BodyCmd.mode = RC_mode::LOCOMOTION`时，下发的前后、左右、旋转速度，才会被执行；`BodyState`接口在任何时候都可以被调用。

- **手柄控制消息结构定义**

  ```c++
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
  ```

  手柄控制为单向控制接口，当然也可以配合获取`BodyState`一起使用。需要注意的是，手柄控制接口控制速度是通过左右推杆推动的深度决定的，并且手柄推杆的原始值范围为-1 ～ 1，因此速度建议推杆结合上下档位键组合使用（档位键不是必需的），参考以下代码处理：

  ```c++
  // 前进速度
  leftStickAnalog[1] = 推杆原始值 × 0.1 × 档位值;
      
  // 当然前进速度也可以直接给值，如0.9m/s
  leftStickAnalog[1] = 0.9;
  ```

  

- **关节控制消息结构定义（预留）**

  该接口一般用不到，并且操作不当容易损坏机器，此为预留接口，有必要时再开通。

  

- **传感器消息结构定义**

  ```c++
  typedef struct
  {
      float   acc[3];                                     // (m/s2)
      float   gyro[3];                                    // angular velocity （rad/s)
      float   rpy[3];                                     // euler angle（rad)
      float   quat[4];                                    // quaternion, normalized, (w,x,y,z)
  } IMU;
  
  typedef struct
  {
      IMU imu;
      int16_t footForce[4];                               // reserve
      int32_t power;                                      // reserve
  } SensorState;
  ```

  目前定义了IMU传感器的原始信息获取，后续可以添加足端传感器、电量等信息的获取。

  

## 2.2. 动态库

动态库安装到`/usr/local/lib/`目录里面，同时还安装了方便链接动态库的cmake文件，应用工程的CMakeLists.txt里面可以通过以下方式链接到sdk的动态库：

```cmake
find_package(bzl_quadruped_sdk REQUIRED)
target_link_libraries(xxx bzl_quadruped_sdk)
```



## 2.3. 示例工程 

- 编译示例工程

  示例放在/opt目录下，需要sudo权限操作，而example本身的所有操作是可以在普通用户下运行的（即SDK本身并不需要sudo权限）

  ```shell
  cd example
  mkdir build 
  cd build
  cmake ..
  make
  ```

- 运行

  注意运行前请确保机器当前情况良好，直接运行编译出来的可执行文件即可。