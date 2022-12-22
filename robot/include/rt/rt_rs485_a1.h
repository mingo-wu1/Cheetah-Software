/*!
 * @file rt_rs485_a1.h
 * @brief rs485 communication to A1 motor
 * @author hanyuanqiang
 */

#ifndef _rt_rs485_a1
#define _rt_rs485_a1

#include <rs485_a1_command_t.hpp>
#include <rs485_a1_data_t.hpp>

void init_rs485_a1();

void rs485_a1_send_receive(rs485_a1_command_t* command, rs485_a1_data_t* data);
void rs485_a1_driver_run();

rs485_a1_data_t* get_rs485_a1_data();
rs485_a1_command_t* get_rs485_a1_command();

#endif

