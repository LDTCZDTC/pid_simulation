import numpy as np
from matplotlib import pyplot as plt

# autor: lijun
# 增量式pid参考链接: https://www.cnblogs.com/zhj66/p/14307385.html


class pid_control_data:
    kp = 0  # pid k参数
    ki = 0  # pid i参数
    kd = 0  # pid d参数
    err = 0  # 系统输出与控制目标的当前误差
    last_err = 0  # 系统输出与控制目标的上一次误差
    llast_eer = 0  # 系统输出与控制目标的上上次误差
    power = 0  # 记录系统输出功耗
    freq = 1000  # 芯片初始频率
    delta = 0  # 控制器增量值
    target_power = 100  # 控制目标
    length_time = 60  # 仿真时间长度
    start_time = 0  # 开始时间
    time = 0  # 时间

    def reset(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0
        self.last_err = 0
        self.llast_eer = 0
        self.power = 0
        self.freq = 1000
        self.delta = 0
        self.target_power = 100
        self.start_time = 0.2 * self.length_time
        self.time = 0
        return


def power_mode(pid_class):  # 被控系统 有两个阶段 一阶段为功耗上升阶段 二阶段为功耗稳定阶段
    if (pid_class.time > pid_class.start_time):
        pid_class.power = 0.16 * pid_class.freq + 25
    else:
        pid_class.power = 0.16 * pid_class.freq * \
            pid_class.time/pid_class.start_time + 25
    return


def pid_control(pid_class):  # pid 控制器核心算法
    pid_class.llast_err = pid_class.last_err
    pid_class.last_err = pid_class.err
    pid_class.err = pid_class.target_power - pid_class.power
    e = pid_class.err
    de = pid_class.err - pid_class.last_err
    dde = de - (pid_class.last_err - pid_class.llast_err)
    pid_p = pid_class.kp * de
    pid_i = pid_class.ki * e
    pid_d = pid_class.kd * dde
    pid_class.delta = pid_p + pid_i + pid_d
    return


def main():
    pcon = pid_control_data()  # 初始化区开始
    length = pcon.length_time
    pid_delta_data = np.zeros(length)
    pid_freq_data = np.zeros(length)
    pid_power_data = np.zeros(length)
    pid_target_power = np.zeros(length)
    x = range(length)
    param_kp = 0.1
    param_ki = 5.4
    param_kd = 0
    fig1 = plt.figure(1)
    fig2 = plt.figure(2)  # 初始化区结束
    for j in range(6):  # 运行六次
        pcon.reset(param_kp, param_ki, param_kd)
        for i in x:  # 运行开始
            power_mode(pcon)
            pid_control(pcon)
            pcon.freq += pcon.delta
            if (pcon.freq > 1100):
                pcon.freq = 1100
            pid_delta_data[i] = pcon.delta
            pid_freq_data[i] = pcon.freq
            pid_power_data[i] = pcon.power
            pid_target_power[i] = pcon.target_power
            pcon.time = pcon.time + 1
            if (pcon.time > 40):
                pcon.target_power = 80  # 运行结束
        else:                                     # 作图区开始
            tempfig = fig1.add_subplot(2, 3, j+1)
            tempfig.set_title("power fig{}      kp:{:.3f} - ki:{:.3f} - kd:{:.3f}".format(
                j, pcon.kp, pcon.ki, pcon.kd))
            tempfig.set_xlabel("time")
            tempfig.set_ylabel("board power")
            tempfig.set_xlim(0, length)
            tempfig.set_ylim(0, 200)
            tempfig.grid()
            tempfig.plot(x, pid_target_power, color='b')
            tempfig.plot(x, pid_power_data, color='g')
            tempfig = fig2.add_subplot(2, 3, j+1)
            tempfig.set_title("freq fig{}      kp:{:.3f} - ki:{:.3f} - kd:{:.3f}".format(
                j, pcon.kp, pcon.ki, pcon.kd))
            tempfig.set_xlabel("time")
            tempfig.set_ylabel("chip freq")
            tempfig.set_xlim(0, length)
            tempfig.set_ylim(0, 1200)
            tempfig.grid()
            tempfig.plot(x, pid_freq_data, color='b')
        # param_ki += 0.2
        param_kp += 0.2
        # param_kd += 0.5
    plt.show()      # 作图区结束， 显示图片
    return


main()
