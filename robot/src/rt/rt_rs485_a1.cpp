/*!
 * @file rt_rs485_a1.cpp
 * @brief rs485 communication to A1 motor
 * @author hanyuanqiang
 */

#include "rt/rt_rs485_a1.h"
#include <lcm/lcm-cpp.hpp>

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/file.h>
#include <sys/time.h> 
#include <math.h>
#include "rt/rs485_a1/LSerial.h"
#include "rt/rs485_a1/motor_ctrl.h"
#include "Logger/Logger.h"

static BZL_QUADRUPED::Logger _logger("RT_RS485_A1");

rs485_a1_command_t rs485_a1_command_drv;
rs485_a1_data_t rs485_a1_data_drv;

#if defined(__linux__)
    int g_fd[4];
#elif defined(__WIN32__)
    HANDLE g_fd[4];
#endif
MOTOR_send motor_send;
MOTOR_recv motor_recv;

pthread_mutex_t rs485_a1_mutex;

#define MAX_CIRCLE_VALUE    (56.52f)
#define REDUCTION_RATIO     (9.1)
#define KP_RATIO            (25)


// const float abad_prone_value[4] = {-0.6f, 0.6f, -0.6f, 0.6f};
// const float hip_prone_value[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
// const float knee_prone_value[4] = {2.7f, 2.7f, 2.7f, 2.7f};
const float abad_prone_value[4] = {-0.554f, 0.554f, -0.554f, 0.554f};
const float hip_prone_value[4] = {-1.137f, -1.137f, -1.137f, -1.137f};
const float knee_prone_value[4] = {2.707f, 2.707f, 2.707f, 2.707f};

// only used for actual robot
const float abad_side_sign[4] = {1.f, 1.f, -1.f, -1.f};
const float hip_side_sign[4] = {1.f, -1.f, 1.f, -1.f};
const float knee_side_sign[4] = {.6333f, -.6333f, .6333f, -.6333f};

// only used for actual robot
float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
float hip_offset[4] = {0.f, 0.f, 0.f, 0.f};
float knee_offset[4] = {0.f, 0.f, 0.f, 0.f};

/*!
 * Init bzl uart
 */
static int bzl_uart_init(char * uartName)
{
    int fd = -1;
    struct termios newtio;

    fd = open(uartName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        return fd;
    }
    bzero( &newtio, sizeof( newtio ) );
    tcgetattr(fd, &newtio);
    cfsetispeed(&newtio, 0010017);
    cfsetospeed(&newtio, 0010017);
    newtio.c_cflag |=  CLOCAL | CREAD | CS8;
    newtio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    newtio.c_oflag = 0;
    newtio.c_oflag &= ~OPOST;
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = sizeof(ServoComdDataV3) - 1;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW,&newtio);

    return fd;
}

/*!
 * It is only used for A1 uart send and recv
 */

static int bzl_uart_send_recv(int fd, char motor_id, MOTOR_send* p_motor_send, MOTOR_recv* p_motor_recv)
{
    static int first_time_leg = 0;
    int read1_num = 0;
    int read2_num = 0;
    int recv_len = 0;
    int send_len = 0;
    int result = -1;

    modify_data(p_motor_send);
    send_len = write(fd, (char *)p_motor_send, sizeof(MasterComdDataV3));
    (void)send_len;
    if (first_time_leg < 12)
    {
        QUADRUPED_INFO(_logger, "Init leg_num[0:3] = %d, motor_id[0:2] = %d", first_time_leg % 4 , motor_id);
        usleep(1000);
    }

    read1_num = read(fd, (char *)p_motor_recv, sizeof(ServoComdDataV3));
    if (read1_num < (int)sizeof(ServoComdDataV3))
    {
        read2_num = read(fd, ((char *)p_motor_recv) + read1_num, sizeof(ServoComdDataV3));
        QUADRUPED_WARN(_logger, "fd:%d, read1_num: %d, read2_num: %d", fd, read1_num, read2_num);
    }
    recv_len = read1_num + read2_num;

    if (sizeof(ServoComdDataV3) == recv_len)
    {
        p_motor_recv->correct = 1;
        p_motor_recv->hex_len = recv_len;
        extract_data(p_motor_recv);
    }
    else
    {
        p_motor_recv->correct = 0;
    }

    if ((motor_id != p_motor_recv->motor_id) || (1 != p_motor_recv->correct) || 
        (p_motor_recv->Pos > MAX_CIRCLE_VALUE) || (p_motor_recv->Pos < -MAX_CIRCLE_VALUE))
    {
        QUADRUPED_ERROR(_logger, 
            "recv_len = %d, motor_id = %d, motor_recv.motor_id = %d, motor_recv.correct = %d, motor_recv.Pos = %f", 
            recv_len, motor_id, p_motor_recv->motor_id, p_motor_recv->correct, p_motor_recv->Pos);
        while (1)
        {
            if (first_time_leg < 12)
            {
                break;
            }
            usleep(100000); 
        }
    }
    else
    {
        result = recv_len;
    }

    if (first_time_leg < 12)
    {
        first_time_leg++;
    }

    return result;
}

/*!
 * Check RS485 A1
 */
static void check_rs485_a1() {
    int error_times = 0;
    struct termios newtio;

    for (uint8_t i = 0; i < 4; i++)
    {
        bzero( &newtio, sizeof( newtio ) );
        tcgetattr(g_fd[i], &newtio);
        newtio.c_cc[VMIN] = 0;
        tcflush(g_fd[i], TCIFLUSH);
        tcsetattr(g_fd[i], TCSANOW,&newtio);
    }

    for (uint8_t motor_id = 0; motor_id < 3; motor_id++)
    {
        for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
        {
            motor_send.mode = 0;
            motor_send.id = motor_id;
            if (-1 == bzl_uart_send_recv(g_fd[leg_num], motor_id, &motor_send, &motor_recv))
            {
                error_times++;
            }
        }
        usleep(1000); 
    }
    

    if (error_times > 0)
    {
        while (1)
        {
            usleep(100000); 
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        bzero( &newtio, sizeof( newtio ) );
        tcgetattr(g_fd[i], &newtio);
        newtio.c_cc[VMIN] = sizeof(ServoComdDataV3) - 1;
        tcflush(g_fd[i], TCIFLUSH);
        tcsetattr(g_fd[i], TCSANOW,&newtio);
    }

    usleep(10000); 
}

/*!
 * Calibration RS485 A1
 */
static void calibration_rs485_a1() {

    for (int motor_id = 0; motor_id < 3; motor_id++)
    {
        for (int leg_num = 0; leg_num < 4; leg_num++)
        {
            motor_send.mode = 0;
            motor_send.id = motor_id;
            bzl_uart_send_recv(g_fd[leg_num], motor_id, &motor_send, &motor_recv);

            switch (motor_id)
            {
            case 0:
                abad_offset[leg_num] = (motor_recv.Pos / REDUCTION_RATIO) - (abad_prone_value[leg_num] * abad_side_sign[leg_num]);
                rs485_a1_data_drv.q_abad[leg_num] = motor_recv.Pos / REDUCTION_RATIO;
                rs485_a1_data_drv.qd_abad[leg_num] = motor_recv.W / REDUCTION_RATIO;
                break;
            case 1:
                hip_offset[leg_num] = (motor_recv.Pos / REDUCTION_RATIO) - (hip_prone_value[leg_num] * hip_side_sign[leg_num]);
                rs485_a1_data_drv.q_hip[leg_num] = motor_recv.Pos / REDUCTION_RATIO;
                rs485_a1_data_drv.qd_hip[leg_num] = motor_recv.W / REDUCTION_RATIO;
                break;
            case 2:
                knee_offset[leg_num] = (motor_recv.Pos / REDUCTION_RATIO) - (knee_prone_value[leg_num] / knee_side_sign[leg_num]);
                rs485_a1_data_drv.q_knee[leg_num] = motor_recv.Pos / REDUCTION_RATIO;
                rs485_a1_data_drv.qd_knee[leg_num] = motor_recv.W / REDUCTION_RATIO;
                break;
            }
        }
        usleep(2000);  
    }
}

/*!
 * Initialize RS485 A1
 */
void init_rs485_a1() {
    QUADRUPED_INFO(_logger, "Init");
    memset(&rs485_a1_command_drv, 0, sizeof(rs485_a1_command_t));
    memset(&rs485_a1_data_drv, 0, sizeof(rs485_a1_data_t));

    if (pthread_mutex_init(&rs485_a1_mutex, NULL) != 0)
    {
        QUADRUPED_ERROR(_logger, "Failed to create rs485 a1 data mutex");
    }

#if defined(__linux__)
    std::string strTtyAXIx;
    for (uint8_t leg_num = 0; leg_num < 4; leg_num++)
    {
        strTtyAXIx = "/dev/ttyAXI" + std::to_string((int)(leg_num + 2));
        g_fd[leg_num] = bzl_uart_init((char *)(strTtyAXIx.c_str()));
    }
    QUADRUPED_INFO(_logger, "Init USE BZL X86 BOARD");
#elif defined(__WIN32__)
    g_fd[0] = open_set((char*)"\\\\.\\COM0");
    g_fd[1] = open_set((char*)"\\\\.\\COM1");
    g_fd[2] = open_set((char*)"\\\\.\\COM2");
    g_fd[3] = open_set((char*)"\\\\.\\COM3");
#endif   

    if ((g_fd[0] < 0) || (g_fd[1] < 0) || (g_fd[2] < 0) || (g_fd[3] < 0))
    {
        QUADRUPED_ERROR(_logger, "Couldn't open RS485");
        BZL_QUADRUPED::LoggerShutDown();
        exit(0);
    }
    else
    {
        check_rs485_a1();
        calibration_rs485_a1();
    }
}

/*!
 * send receive data and command from A1 motor
 */
void rs485_a1_send_receive(rs485_a1_command_t* command, rs485_a1_data_t* data) {

    static int rs548_a1_driver_iterations = 0;

    rs548_a1_driver_iterations++;
    data->rs485_a1_driver_status = rs548_a1_driver_iterations << 16;

    for (uint8_t motor_id = 0; motor_id < 3; motor_id++)
    {
        for (int leg_num = 0; leg_num < 4; leg_num++)
        {
            motor_send.id = motor_id;
            if (1 == command->flags[leg_num])
            {
                motor_send.mode = 10;
            }
            else
            {
                motor_send.mode = 0;
            }
            
            switch (motor_id)
            {
            case 0:
                motor_send.Pos = ((command->q_des_abad[leg_num] * abad_side_sign[leg_num]) + abad_offset[leg_num]) * REDUCTION_RATIO;
                motor_send.W = command->qd_des_abad[leg_num] * abad_side_sign[leg_num] * REDUCTION_RATIO;
                motor_send.K_P = command->kp_abad[leg_num];
                motor_send.K_W = command->kd_abad[leg_num];
                motor_send.T = command->tau_abad_ff[leg_num] * abad_side_sign[leg_num] / REDUCTION_RATIO;
                break;
            case 1:
                motor_send.Pos = ((command->q_des_hip[leg_num] * hip_side_sign[leg_num]) + hip_offset[leg_num]) * REDUCTION_RATIO;
                motor_send.W = command->qd_des_hip[leg_num] * hip_side_sign[leg_num] * REDUCTION_RATIO;
                motor_send.K_P = command->kp_hip[leg_num];
                motor_send.K_W = command->kd_hip[leg_num];
                motor_send.T = command->tau_hip_ff[leg_num] * hip_side_sign[leg_num] / REDUCTION_RATIO;
                break;
            case 2:
                motor_send.Pos = ((command->q_des_knee[leg_num] / knee_side_sign[leg_num]) + knee_offset[leg_num]) * REDUCTION_RATIO;
                motor_send.W = command->qd_des_knee[leg_num] / knee_side_sign[leg_num] * REDUCTION_RATIO;
                motor_send.K_P = command->kp_knee[leg_num];
                motor_send.K_W = command->kd_knee[leg_num];
                motor_send.T = command->tau_knee_ff[leg_num] * knee_side_sign[leg_num] / REDUCTION_RATIO;
                break;
            }
            
#if 1
            {
                if ((motor_send.K_P > 2) || (motor_send.K_P < 0))
                {
                    QUADRUPED_ERROR(_logger, "motor_send.K_P = %f", motor_send.K_P);
                    motor_send.K_P = 0;
                }

                if ((motor_send.K_W > 2) || (motor_send.K_W < 0))
                {
                    QUADRUPED_ERROR(_logger, "motor_send.K_W = %f", motor_send.K_W);
                    motor_send.K_W = 0;
                }

                if (motor_send.T > (10) || motor_send.T < (-10))
                {
                    QUADRUPED_WARN(_logger, "1 motor_send.T = %f", motor_send.T);
                    motor_send.T = 0;
                }
                else if (motor_send.T > 2)
                {                               // 限力
                    QUADRUPED_WARN(_logger, "2 motor_send.T = %f", motor_send.T);
                    motor_send.T = 2;
                }
                else if (motor_send.T < -2)
                {
                    QUADRUPED_WARN(_logger, "3 motor_send.T = %f", motor_send.T);
                    motor_send.T = -2;
                }
            }
#endif
            motor_send.K_P /= KP_RATIO;
            
            bzl_uart_send_recv(g_fd[leg_num], motor_id, &motor_send, &motor_recv);

            data->flags[leg_num] = (motor_recv.mode != 0) ? 1 : 0;  // TODO(hanyuanqiang): 2021-03-27, 0 = 0, other = 1?
            switch (motor_id)
            {
            case 0:
                data->q_abad[leg_num] = (motor_recv.Pos / REDUCTION_RATIO - abad_offset[leg_num]) * abad_side_sign[leg_num];
                data->qd_abad[leg_num] = motor_recv.W / REDUCTION_RATIO * abad_side_sign[leg_num];
                data->temp_abad[leg_num] = motor_recv.Temp;
                data->tau_abad[leg_num] = motor_recv.T * REDUCTION_RATIO * abad_side_sign[leg_num];
                data->acc_abad[leg_num] = motor_recv.Acc / REDUCTION_RATIO * abad_side_sign[leg_num];
                break;
            case 1:
                data->q_hip[leg_num] = (motor_recv.Pos / REDUCTION_RATIO - hip_offset[leg_num]) * hip_side_sign[leg_num];
                data->qd_hip[leg_num] = motor_recv.W / REDUCTION_RATIO * hip_side_sign[leg_num];
                data->temp_hip[leg_num] = motor_recv.Temp;
                data->tau_hip[leg_num] = motor_recv.T * REDUCTION_RATIO * hip_side_sign[leg_num];
                data->acc_hip[leg_num] = motor_recv.Acc / REDUCTION_RATIO * hip_side_sign[leg_num];
                break;
            case 2:
                data->q_knee[leg_num] = (motor_recv.Pos / REDUCTION_RATIO - knee_offset[leg_num]) * knee_side_sign[leg_num];
                data->qd_knee[leg_num] = motor_recv.W / REDUCTION_RATIO * knee_side_sign[leg_num];
                data->temp_knee[leg_num] = motor_recv.Temp;
                data->tau_knee[leg_num] = motor_recv.T * REDUCTION_RATIO / knee_side_sign[leg_num];
                data->acc_knee[leg_num] = motor_recv.Acc / REDUCTION_RATIO * knee_side_sign[leg_num];
                break;
            default:
                break;
            }
        }
    }
}

/*!
 * Run RS485 A1
 */
void rs485_a1_driver_run() {
    // in here, the driver is good
    pthread_mutex_lock(&rs485_a1_mutex);
    rs485_a1_send_receive(&rs485_a1_command_drv, &rs485_a1_data_drv);
    pthread_mutex_unlock(&rs485_a1_mutex);
}

/*!
 * Get the rs485 a1 command
 */
rs485_a1_command_t *get_rs485_a1_command() {
   return &rs485_a1_command_drv;
}

/*!
 * Get the rs485 a1 data
 */
rs485_a1_data_t *get_rs485_a1_data() {
    return &rs485_a1_data_drv;
}

