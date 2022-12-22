#!/usr/bin/python3
# -*- coding: utf-8 -*-
# @Time : 2021-01-28
# @Author : wuchunming02@countrygarden.com.cn
# @Function : publish lcm data of spi command

import lcm
import time

from test_spi_command_t import test_spi_command_t

from sys import argv

distance = argv[1]

lc = lcm.LCM()

msg = test_spi_command_t()

msg.is_test = 1

msg.q_des_abad = (0+float(distance), 0+float(distance), 0+float(distance), 0+float(distance))
msg.q_des_hip =  (0+float(distance), 0+float(distance), 0+float(distance), 0+float(distance))
msg.q_des_knee = (0+float(distance), 0+float(distance), 0+float(distance), 0+float(distance))

msg.qd_des_abad = (1, 1, 1, 1)
msg.qd_des_hip =  (1, 1, 1, 1)
msg.q_des_knee =  (1, 1, 1, 1)

msg.kp_abad = (1.2, 1.2, 1.2, 1.2)
msg.kp_hip =  (1.2, 1.2, 1.2, 1.2)
msg.kp_knee = (1.2, 1.2, 1.2, 1.2)

msg.kd_abad = (0.3, 0.3, 0.3, 0.3)
msg.kd_hip =  (0.3, 0.3, 0.3, 0.3)
msg.kd_knee = (0.3, 0.3, 0.3, 0.3)

msg.tau_abad_ff = (0, 0, 0, 0)
msg.tau_hip_ff =  (0, 0, 0, 0)
msg.tau_knee_ff = (0, 0, 0, 0)

msg.flags = (1, 1, 1, 1)

while(True):
  time.sleep(3)
  msg.q_des_abad = (0+float(distance), 0+float(distance), 0+float(distance), 0+float(distance))
  msg.q_des_hip =  (0+float(distance), 0+float(distance), 0+float(distance), 0+float(distance))
  msg.q_des_knee = (0+float(distance), 0+float(distance), 0+float(distance), 0+float(distance))
  lc.publish("testspicmd", msg.encode())
