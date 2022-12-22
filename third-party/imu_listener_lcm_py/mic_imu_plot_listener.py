#!/usr/bin/python3
# -*- coding: utf-8 -*-
# @Time : 2021-01-22
# @Author : wuchunming02@countrygarden.com.cn
# @Function : listener lcm data of microstrain imu

import lcm
from microstrain_lcmt import microstrain_lcmt
import matplotlib.pyplot as plt
import threading
import time

rpys = [(0,0,0),]

def my_handler(channel, data):
    mic = microstrain_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   mic->q_des_abad   = %s" % str(mic.quat))
    print("   mic->q_des_hip	= %s" % str(mic.rpy))
    print("   mic->q_des_knee   = %s" % str(mic.omega))
    print("   mic->qd_des_abad  = %s" % str(mic.acc))
    print("---")
    rpys.append(mic.rpy)

lc = lcm.LCM()
subscription = lc.subscribe("microstrain", my_handler)

def runlcm(lc):
    while True:
        lc.handle()

def runplt(rpys,plt,pauseTime):
    while True:
        rl = []
        pl = []
        yl = []
        for rpy in rpys:
            rl.append(rpy[0])
            pl.append(rpy[1])
            yl.append(rpy[2])
        plt.subplot(3,1,1)
        plt.plot(rl, c = 'red', label = 'IMU')
        plt.ylabel('r')
        plt.title('Microstrain IMU RPY')
        plt.subplot(3,1,2)
        plt.plot(pl, c = 'green', label = 'IMU')
        plt.ylabel('p')
        plt.subplot(3,1,3)
        plt.plot(pl, c = 'blue', label = 'IMU')
        plt.ylabel('y')
        plt.pause(pauseTime)
        plt.cla()

try:
    t_lcm = threading.Thread(target = runlcm,args = (lc, ))
    t_lcm.start()
    t_spi_plt = threading.Thread(target = runplt,args = (rpys, plt, 0.05, ))
    t_spi_plt.start()
    t_spi_plt.join()

except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
