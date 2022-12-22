#!/usr/bin/python3
# -*- coding: utf-8 -*-
# @Time : 2021-01-22
# @Author : wuchunming02@countrygarden.com.cn
# @Function : subscribe lcm data of spi command

import lcm
from spi_command_t import spi_command_t

def my_handler(channel, data):
    cmds = spi_command_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
#    print("   timestamp   = %s" % str(msg.timestamp))
#    print("   position    = %s" % str(msg.position))
#    print("   orientation = %s" % str(msg.orientation))
#    print("   ranges: %s" % str(msg.ranges))
#    print("   name        = '%s'" % msg.name)
#    print("   enabled     = %s" % str(msg.enabled))
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

lc = lcm.LCM()
subscription = lc.subscribe("spi_command", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
