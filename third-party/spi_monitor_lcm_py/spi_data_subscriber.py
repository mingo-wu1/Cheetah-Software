 #!/usr/bin/python3
import lcm
from spi_command_t import spi_command_t
from spi_data_t import spi_data_t
import matplotlib.pyplot as plt
import threading
import time
import os

q_des_abads = [(0,0,0,0),]#ab/ad电机角度期望
q_des_hip = [(0,0,0,0),]#髋关节电机角度期望
q_des_knee = [(0,0,0,0),]#膝关节电机角度
qd_des_abad = [(0,0,0,0),]#ab/ad电机角速度期望
qd_des_hip = [(0,0,0,0),]#髋关节电机角速度期望
qd_des_knee = [(0,0,0,0),]#膝关节电机角速度期望
kp_abad = [(0,0,0,0),]#ab/ad电机kp参数
kp_hip = [(0,0,0,0),]#髋关节电机kp参数
kp_knee = [(0,0,0,0),]#膝关节电机kp参数
kd_abad = [(0,0,0,0),]#ab/ad电机kd参数
kd_hip = [(0,0,0,0),]#髋关节电机kd参数
kd_knee = [(0,0,0,0),]#膝关节电机kd参数
tau_abad_ff = [(0,0,0,0),]#ab/ad电机力矩参数
tau_hip_ff = [(0,0,0,0),]#髋关节电机力矩参数
tau_knee_ff = [(0,0,0,0),]#膝关节电机力矩参数
flags = [0,]#使能标志

#path = r"./log_spi_cmd.txt"
#if os.path.exists(path) is False:
#    os.mknod(path)
#f_cmd = open(path,"a",encoding="utf-8")

def spi_cmd_handler(channel, data):
    cmds = spi_command_t.decode(data)
    print("Received spi_cmd message on channel \"%s\"" % channel)
    print("cmd->q_des_abad(四个ab/ad电机角度期望)   = %s" % str(cmds.q_des_abad))
    print("cmd->q_des_hip(四个髋关节电机角度期望)	= %s" % str(cmds.q_des_hip))
    print("cmd->q_des_knee(四个膝关节电机角度期望)   = %s" % str(cmds.q_des_knee))
    print("cmd->qd_des_abad(四个ab/ad电机角速度期望)  = %s" % str(cmds.qd_des_abad))
    print("cmd->qd_des_hip(四个髋关节电机角速度期望)   = %s" % str(cmds.qd_des_hip))
    print("cmd->qd_des_knee(四个膝关节电机角速度期望)  = %s" % str(cmds.qd_des_knee))
    print("cmd->kp_abad(四个ab/ad电机kp参数)   	= %s" % str(cmds.kp_abad))
    print("cmd->kp_hip(四个髋关节电机kp参数)   	= %s" % str(cmds.kp_hip))
    print("cmd->kp_knee(四个膝关节电机kp参数)   	= %s" % str(cmds.kp_knee))
    print("cmd->kd_abad(四个ab/ad电机kd参数)   	= %s" % str(cmds.kd_abad))
    print("cmd->kd_hip(四个髋关节电机kd参数)   	= %s" % str(cmds.kd_hip))
    print("cmd->kd_knee(四个膝关节电机kd参数)   	= %s" % str(cmds.kd_knee))
    print("cmd->tau_abad_ff(四个ab/ad电机力矩参数)  = %s" % str(cmds.tau_abad_ff))
    print("cmd->tau_hip_ff(四个髋关节电机力矩参数)   = %s" % str(cmds.tau_hip_ff))
    print("cmd->tau_knee_ff(四个膝关节电机力矩参数)  = %s" % str(cmds.tau_knee_ff))
    print("cmd->flags(四个使能标志)   	= %s" % str(cmds.flags))
    print("---")
#    f_cmd.write(str(cmds.q_des_abad[0])+" "+str(cmds.q_des_abad[1])+" "+str(cmds.q_des_abad[2])+" "+str(cmds.q_des_abad[3])+" ")
#    f_cmd.write(str(cmds.q_des_hip[0])+" "+str(cmds.q_des_hip[1])+" "+str(cmds.q_des_hip[2])+" "+str(cmds.q_des_hip[3])+" ")
#    f_cmd.write(str(cmds.q_des_knee[0])+" "+str(cmds.q_des_knee[1])+" "+str(cmds.q_des_knee[2])+" "+str(cmds.q_des_knee[3])+" ")
#    f_cmd.write(str(cmds.qd_des_abad[0])+" "+str(cmds.qd_des_abad[1])+" "+str(cmds.qd_des_abad[2])+" "+str(cmds.qd_des_abad[3])+" ")
#    f_cmd.write(str(cmds.qd_des_hip[0])+" "+str(cmds.qd_des_hip[1])+" "+str(cmds.qd_des_hip[2])+" "+str(cmds.qd_des_hip[3])+" ")
#    f_cmd.write(str(cmds.qd_des_knee[0])+" "+str(cmds.qd_des_knee[1])+" "+str(cmds.qd_des_knee[2])+" "+str(cmds.qd_des_knee[3])+" ")
#    f_cmd.write(str(cmds.tau_abad_ff[0])+" "+str(cmds.tau_abad_ff[1])+" "+str(cmds.tau_abad_ff[2])+" "+str(cmds.tau_abad_ff[3])+" ")
#    f_cmd.write(str(cmds.tau_hip_ff[0])+" "+str(cmds.tau_hip_ff[1])+" "+str(cmds.tau_hip_ff[2])+" "+str(cmds.tau_hip_ff[3])+" ")
#    f_cmd.write(str(cmds.tau_knee_ff[0])+" "+str(cmds.tau_knee_ff[1])+" "+str(cmds.tau_knee_ff[2])+" "+str(cmds.tau_knee_ff[3])+" ")
#    f_cmd.write("\n")
#    q_des_abads.append(cmds.q_des_abad)

q_abads=[(0,0,0,0),]#ab/ad电机角度反馈
q_hips=[(0,0,0,0),]#髋关节电机角度反馈
q_knees=[(0,0,0,0),]#膝关节电机角度反馈
qd_abads=[(0,0,0,0),]#ab/ad电机角速度反馈
qd_hips=[(0,0,0,0),]#髋关节电机角速度反馈
qd_knees=[(0,0,0,0),]#膝关节电机角速度反馈
flagss=[0,]#使能标志
spi_driver_statuss=[0,]#SPI驱动状态

#path = r"./log_spi_data.txt"
#if os.path.exists(path) is False:
#    os.mknod(path)
#f = open(path,"a",encoding="utf-8")

def spi_data_handler(channel, data):
    datas = spi_data_t.decode(data)
    print("Received spi_data message on channel \"%s\"" % channel)
    print("datas->q_des_abad(四个ab/ad电机角度反馈)   = %s" % str(datas.q_abad))
    print("datas->q_des_hip(四个髋关节电机角度反馈)	= %s" % str(datas.q_hip))
    print("datas->q_des_knee(四个膝关节电机角度反馈)   = %s" % str(datas.q_knee))
    print("datas->qd_des_abad(四个ab/ad电机角速度反馈)  = %s" % str(datas.qd_abad))
    print("datas->qd_des_hip(四个髋关节电机角速度反馈)   = %s" % str(datas.qd_hip))
    print("datas->qd_des_knee(四个膝关节电机角速度反馈)  = %s" % str(datas.qd_knee))
    print("datas->tau_abad(四个ab/ad电机力矩反馈) = %s" % str(datas.tau_abad)")
    print("datas->tau_hip(四个髋关节电机力矩反馈) = %s" % str(datas.tau_hip)")
    print("datas->tau_knee(四个膝关节电机力矩反馈) = %s" % str(datas.tau_knee)")
    print("datas->flags(四个使能标志)   	= %s" % str(datas.flags))
    print("datas->spi_driver_status(四个SPI驱动状态)   	= %s" % str(datas.spi_driver_status))
    print("---")
#    f.write(str(datas.q_abad[0])+" "+str(datas.q_abad[1])+" "+str(datas.q_abad[2])+" "+str(datas.q_abad[3])+" ")
#    f.write(str(datas.q_hip[0])+" "+str(datas.q_hip[1])+" "+str(datas.q_hip[2])+" "+str(datas.q_hip[3])+" ")
#    f.write(str(datas.q_knee[0])+" "+str(datas.q_knee[1])+" "+str(datas.q_knee[2])+" "+str(datas.q_knee[3])+" ")
#    f.write(str(datas.qd_abad[0])+" "+str(datas.qd_abad[1])+" "+str(datas.qd_abad[2])+" "+str(datas.qd_abad[3])+" ")
#    f.write(str(datas.qd_hip[0])+" "+str(datas.qd_hip[1])+" "+str(datas.qd_hip[2])+" "+str(datas.qd_hip[3])+" ")
#    f.write(str(datas.qd_knee[0])+" "+str(datas.qd_knee[1])+" "+str(datas.qd_knee[2])+" "+str(datas.qd_knee[3])+" ")
#    f.write("\n")
#    for i in range(50):
#        q_des_abads.append(datas.q_abad)
#        q_hips.append(datas.q_hip)
#        q_knees.append(datas.q_knee)
#        qd_abads.append(datas.qd_abad)
#        qd_hips.append(datas.qd_hip)
#        qd_knees.append(datas.qd_knee)
#        if i >= 50:
#            q_des_abads.pop()
#            q_hips.pop()
#            q_knees.pop()
#            qd_abads.pop()
#            qd_hips.pop()
#            qd_knees.pop()

lc = lcm.LCM()
#subscription = lc.subscribe("spi_command", spi_cmd_handler)
subscription = lc.subscribe("spi_data", spi_data_handler)

def runlcm(lc):
    while True:
        lc.handle()

def runplt(legID, q_abads, q_hips, q_knees, qd_abads, qd_hips, qd_knees, plt, pauseTime):
    mng=plt.get_current_fig_manager()
    mng.resize(*mng.window.maxsize())
    while True:
        q_abads_l = []
        q_hips_l = []
        q_knees_l = []
        qd_abads_l = []
        qd_hips_l = []
        qd_knees_l = []
        for i in range(50):
            q_abads_l.append(q_abads[0])
            q_hips_l.append(q_hips[0])
            q_knees_l.append(q_knees[0])
            qd_abads_l.append(qd_abads[0])
            qd_hips_l.append(qd_hips[0])
            qd_knees_l.append(qd_knees[0])
        ###
        plt.subplot(3,2,1)
        plt.plot(q_abads_l, c = 'red', label = 'ab/ad电机角度反馈')
        plt.ylabel('rad')
        plt.title('ab/ad Motor angle feedback')
        ###
        plt.subplot(3,2,2)
        plt.plot(q_hips_l, c = 'green', label = '髋关节电机角度反馈')
        plt.ylabel('rad')
        plt.title('hip joint Motor angle feedback')
        ###
        plt.subplot(3,2,3)
        plt.plot(q_knees_l, c = 'blue', label = '膝关节电机角度反馈')
        plt.ylabel('rad')
        plt.title('knee joint Motor angle feedback')
        ###
        plt.subplot(3,2,4)
        plt.plot(qd_abads_l, c = 'blue', label = 'ab/ad电机角速度反馈')
        plt.ylabel('rad/s')
        plt.title('ab/ad Motor angular velocity feedback')
        ###
        plt.subplot(3,2,5)
        plt.plot(qd_hips_l, c = 'blue', label = '髋关节电机角速度反馈')
        plt.ylabel('rad/s')
        plt.title('hip knee Motor angular velocity feedback')
        ###
        plt.subplot(3,2,6)
        plt.plot(qd_knees_l, c = 'blue', label = '膝关节电机角速度反馈')
        plt.ylabel('rad/s')
        plt.title('knee joint Motor angular velocity feedback')
        ###
        plt.pause(pauseTime)
        plt.cla()

try:
    t_lcm = threading.Thread(target = runlcm,args = (lc, ))
    t_lcm.start()
    t_lcm.join()
#    t_spi_plt = threading.Thread(target = runplt,args = (0, q_abads, q_hips, q_knees, qd_abads, qd_hips, qd_knees, plt, 0.05, ))
#    t_spi_plt.start()
#    t_spi_plt.join()

except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
#f.close()
#f_cmd.close()
