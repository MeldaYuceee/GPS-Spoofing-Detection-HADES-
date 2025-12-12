import numpy as np
import matplotlib.pyplot as plt
import csv

# Basit drone state yapısı
class DroneState:
    def __init__(self, x=0.0, y=0.0, vx=5.0, vy=0.0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy


# GPS spoofing tespit modülü
class HADESDetector:
    def __init__(self, max_speed=25.0, innovation_limit=30.0, heading_thresh=45):
        self.max_speed = max_speed
        self.innovation_limit = innovation_limit
        self.heading_limit = np.deg2rad(heading_thresh)

        self.prev_gps = None
        self.prev_est = None
        self.prev_heading = None

    def update(self, gps_pos, est_pos, dt):
        flags = {"speed_jump": False, "innovation": False, "heading_jump": False}

        # GPS speed jump
        if self.prev_gps is not None:
            dx = gps_pos[0] - self.prev_gps[0]
            dy = gps_pos[1] - self.prev_gps[1]
            speed = np.sqrt(dx*dx + dy*dy) / dt
            if speed > self.max_speed:
                flags["speed_jump"] = True

        # Heading jump
        heading = np.arctan2(gps_pos[1], gps_pos[0])
        if self.prev_heading is not None:
            diff = abs(heading - self.prev_heading)
            if diff > self.heading_limit:
                flags["heading_jump"] = True
        self.prev_heading = heading

        # GPS vs IMU farkı
        if self.prev_est is not None:
            innovation = np.linalg.norm(gps_pos - est_pos)
            if innovation > self.innovation_limit:
                flags["innovation"] = True

        self.prev_gps = gps_pos.copy()
        self.prev_est = est_pos.copy()

        spoof = flags["speed_jump"] or flags["innovation"] or flags["heading_jump"]
        return spoof, flags



# Fail-safe kaçış kontrolü
class FailSafeController:
    def __init__(self, home_x=0, home_y=0, escape_speed=8):
        self.mode = "NORMAL"
        self.home = np.array([home_x, home_y], float)
        self.escape_speed = escape_speed

    def update_mode(self, detected):
        if self.mode == "NORMAL" and detected:
            self.mode = "ESCAPE"
        return self.mode

    def command(self, est_pos):
        if self.mode == "NORMAL":
            return np.array([5.0, 0.0])

        d = self.home - est_pos
        dist = np.linalg.norm(d)
        if dist < 1:
            return np.array([0.0, 0.0])

        d = d / dist
        return d * self.escape_speed



# Ana simülasyon
def simulate_hades():
    dt = 0.5
    T = 120
    steps = int(T / dt)

    drone = DroneState()
    detector = HADESDetector()
    controller = FailSafeController()

    est_pos = np.array([0.0, 0.0], float)

    spoof_start = 40
    spoof_end = 80
    spoof_shift = np.array([300.0, 200.0])

    time_list = []
    true_path = []
    gps_path = []
    est_path = []
    modes = []

    for i in range(steps):
        t = i * dt

        # gerçek hareket
        drone.x += drone.vx * dt
        drone.y += drone.vy * dt
        true_pos = np.array([drone.x, drone.y])

        # IMU dead reckoning
        est_pos = est_pos + np.array([drone.vx, drone.vy]) * dt + np.random.randn(2) * 0.8

        # GPS
        gps = true_pos + np.random.randn(2) * 2
        if spoof_start <= t <= spoof_end:
            gps = gps + spoof_shift

        # tespit
        spoofed, flags = detector.update(gps, est_pos, dt)
        mode = controller.update_mode(spoofed)

        # hız komutu
        cmd = controller.command(est_pos)
        drone.vx, drone.vy = cmd[0], cmd[1]

        # log
        time_list.append(t)
        true_path.append(true_pos)
        gps_path.append(gps)
        est_path.append(est_pos)
        modes.append(mode)

        if i % 5 == 0:
            print(f"{t:.1f}s | mode={mode} | flags={flags}")

    # array'e çevir
    true_path = np.array(true_path)
    gps_path = np.array(gps_path)
    est_path = np.array(est_path)

    # CSV LOG KAYDI
    with open("hades_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "true_x", "true_y", "gps_x", "gps_y", "imu_x", "imu_y", "mode"])
        for i in range(len(time_list)):
            w.writerow([time_list[i],
                        true_path[i][0], true_path[i][1],
                        gps_path[i][0], gps_path[i][1],
                        est_path[i][0], est_path[i][1],
                        modes[i]])

    # grafik
    plt.figure(figsize=(8, 6))
    plt.plot(true_path[:,0], true_path[:,1], label="True Path")
    plt.plot(gps_path[:,0], gps_path[:,1], ".", alpha=0.4, label="GPS")
    plt.plot(est_path[:,0], est_path[:,1], label="IMU Est.")

    if "ESCAPE" in modes:
        idx = modes.index("ESCAPE")
        x, y = true_path[idx]
        plt.scatter(x, y, s=100, marker="x", label="ESCAPE Start")
        plt.axvline(x, color="red", linestyle="--", linewidth=1)

    plt.legend()
    plt.grid()
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("HADES-X Spoofing Detection Simulation")
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    simulate_hades()
