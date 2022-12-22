#!/usr/bin/python3
# -*- coding: utf-8 -*-
# @Time : 2021-01-22
# @Author : wuchunming02@countrygarden.com.cn
# @Function : subscribe lcm data of spi command

import lcm
from spi_command_t import spi_command_t
import matplotlib.pyplot as plt
import threading
import time

q_des_abads = [(0,0,0,0),]
def my_handler(channel, data):
    cmds = spi_command_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   cmd->q_des_abad   = %s" % str(cmds.q_des_abad))
    print("   cmd->q_des_hip	= %s" % str(cmds.q_des_hip))
    print("   cmd->q_des_knee   = %s" % str(cmds.q_des_knee))
    print("   cmd->qd_des_abad  = %s" % str(cmds.qd_des_abad))
    print("   cmd->qd_des_hip   = %s" % str(cmds.qd_des_hip))
    print("   cmd->qd_des_knee  = %s" % str(cmds.qd_des_knee))
    print("   cmd->kp_abad   	= %s" % str(cmds.kp_abad))
    print("   cmd->kp_hip   	= %s" % str(cmds.kp_hip))
    print("   cmd->kp_knee   	= %s" % str(cmds.kp_knee))
    print("   cmd->kd_abad   	= %s" % str(cmds.kd_abad))
    print("   cmd->kd_hip   	= %s" % str(cmds.kd_hip))
    print("   cmd->kd_knee   	= %s" % str(cmds.kd_knee))
    print("   cmd->tau_abad_ff  = %s" % str(cmds.tau_abad_ff))
    print("   cmd->tau_hip_ff   = %s" % str(cmds.tau_hip_ff))
    print("   cmd->tau_knee_ff  = %s" % str(cmds.tau_knee_ff))
    print("   cmd->flags   	= %s" % str(cmds.flags))
    print("---")
    q_des_abads.append(cmds.q_des_abad)

lc = lcm.LCM()
subscription = lc.subscribe("spi_command", my_handler)

def runlcm(lc):
    while True:
        lc.handle()

def runplt(q_des_abads,index,plt,pauseTime):
    while True:
        l = []
        for q_des_abad in q_des_abads:
            l.append(q_des_abad[index])
        plt.plot(l, c = 'red', label = 'IMU')        
        plt.pause(pauseTime)
        plt.cla()

try:
    t_lcm = threading.Thread(target = runlcm,args = (lc, ))
    t_lcm.start()
    t_spi_plt = threading.Thread(target = runplt,args = (q_des_abads, 0, plt, 0.05, ))
    t_spi_plt.start()
    t_spi_plt.join()

except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
