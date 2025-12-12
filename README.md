# GPS-Spoofing-Detection-HADES
# 🛡️ HADES  
## GPS Spoofing Detection & Fail-Safe Navigation Prototype

HADES is a **beginner-level but defense-oriented Python prototype** that explores how a UAV might detect **GPS spoofing** and react safely when navigation data becomes untrustworthy.

This project was built as part of my **Week 1 learning series** focused on UAV safety, reliability, and failure modes.  
It is **not a production system**, but a simplified model to understand how trust in navigation data can break — and how a system should respond.

---

## 📌 Project Motivation

In UAV systems, GPS data can look perfectly valid while being completely wrong.  
Unlike jamming, spoofing is quiet, subtle, and dangerous.

If a drone blindly trusts GPS:
- It may drift away from its real position
- Autopilot logic remains “correct” while perception is false
- The system fails without realizing it

HADES was built to explore a simple question:

> *How can a UAV decide that GPS data no longer makes physical sense?*

---

## 🧠 Core Idea

HADES compares two independent sources of position information:

- **GPS position** (can be spoofed)
- **Physics-based estimate** (IMU / dead-reckoning style)

Instead of complex sensor fusion, HADES uses **basic sanity checks**:

- Is the GPS-implied speed physically possible?
- Is the position jump consistent with expected motion?
- Does GPS drift too far from the internal estimate?

If trust breaks, GPS is marked as **SPOOFED**.

---

## ⚙️ Detection Logic (Simplified)

Each timestep, HADES checks:

1. **Speed plausibility**
   - Unrealistic velocity → suspicious

2. **Position consistency**
   - Large divergence between GPS and IMU estimate → suspicious

If any rule is violated:
- GPS status → `SPOOFED`
- Navigation switches to fallback mode

This mirrors a real-world principle:
> *When a sensor becomes untrustworthy, stop using it.*

---

## 🛑 Fail-Safe Behavior

HADES includes a minimal fail-safe reaction:

- **GPS OK** → use GPS normally  
- **GPS SPOOFED** → ignore GPS completely  
  → rely only on the internal motion estimate

The goal is not perfect navigation, but **predictable behavior under failure**.

---

## 🧪 Simulation Overview

- Initial phase: GPS and IMU agree
- Spoofing phase: GPS position drifts unnaturally
- Detection: HADES flags spoofing
- Reaction: system switches to fallback navigation

Even in this small simulation, the moment when the system “loses trust” in GPS becomes very clear.

---

## 📂 Project Structure

HADES/
├── src/
│ ├── drone.py # Drone motion model
│ ├── gps.py # GPS (normal + spoofed)
│ ├── hades.py # Spoofing detection logic
│ └── simulation.py # Main loop
│
├── graphic-photo/
│ └── hades_demo.png # Simulation visualization
│
├── requirements.txt
└── README.md

---

## 🖼️ Example Output

Below is a visualization from the simulation, showing how GPS drift and detection evolve over time:

![HADES Simulation Output](https://raw.githubusercontent.com/MeldaYuceee/GPS-Spoofing-Detection-HADES-/main/graphic-photo/graphic-photo.png)

---

## 🚀 How to Run

```bash
pip install -r requirements.txt
python src/simulation.py
