import numpy as np
from matplotlib import pyplot as plt


class pid_control_data:
    kp = 0
    ki = 0
    kd = 0
    err = 0
    last_err = 0
    llast_eer = 0
    power = 0
    freq = 1000
    delta = 0
    target_power = 100
    length_time = 60
    start_time = 0
    time = 0

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


def power_mode(pid_class):
    if (pid_class.time > pid_class.start_time):
        pid_class.power = 0.16 * pid_class.freq + 25
    else:
        pid_class.power = 0.16 * pid_class.freq * \
            pid_class.time/pid_class.start_time + 25
    return


def pid_control(pid_class):
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
    pcon = pid_control_data()
    length = pcon.length_time
    pid_delta_data = np.zeros(length)
    pid_freq_data = np.zeros(length)
    pid_power_data = np.zeros(length)
    pid_target_power = np.zeros(length)
    x = range(length)
    param_kp = 0.2
    param_ki = 5.3
    param_kd = 0
    fig1 = plt.figure(1)
    fig2 = plt.figure(2)
    for j in range(6):
        pcon.reset(param_kp, param_ki, param_kd)
        for i in x:
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
                pcon.target_power = 80
        else:
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
        param_ki += 0.1
    plt.show()
    return


main()
