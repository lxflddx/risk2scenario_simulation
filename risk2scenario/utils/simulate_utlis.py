import numpy as np
import matplotlib.pyplot as plt


def plot_speed_graph (ego_speed_list):
    print("Drawing the speed graphs...")
    time = np.arange(len(ego_speed_list)) / 20.0  # 时间轴（单位：秒）
    plt.figure(figsize=(10, 6))
    plt.plot(time, ego_speed_list, label="Ego Vehicle Speed (km/h)")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (km/h)")
    plt.title("Ego Vehicle Speed Over Time")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_acceleration_graph (ego_acc_list):
    print("Drawing the acceleration graphs...")
    time = np.arange(len(ego_acc_list)) / 20.0  # 时间轴（单位：秒）
    plt.figure(figsize=(10, 6))
    plt.plot(time, ego_acc_list, label="Ego Vehicle Acceleration (m/s^2)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title("Ego Vehicle Acceleration Over Time")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_ttc_graph (ttc_list):
    print("Drawing the acceleration graphs...")
    time = np.arange(len(ttc_list)) / 20.0  # 时间轴（单位：秒）
    plt.figure(figsize=(10, 6))
    plt.plot(time, ttc_list, label="Ego Vehicle Acceleration (m/s^2)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title("Ego Vehicle Acceleration Over Time")
    plt.legend()
    plt.grid(True)
    plt.show()


def acc_check (acc_list):
    timestamps = len(acc_list)

    if timestamps < 2:
        return 0
    alpha = 1.0
    count = 0

    for i in range(1, timestamps):

        if abs(
                acc_list [i]
                - acc_list [i - 1]
        ) >= alpha:
            count += 1

    acr = count / timestamps

    return acr
