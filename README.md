# 5-DOF Robotic Arm

<img width="562.5" height="750" alt="20260819_220313" src="https://github.com/user-attachments/assets/e7e38184-a4f2-48fd-8ef7-fed7db602751" />

I built a custom 5-degree-of-freedom robotic arm, driven by 2 20:1 cycloidal drives and 3 servos. The system achieves a 440mm reach, wih accurate inverse kinematics (IK) and trajectory planning. This setup delivers the reach and ease of use necessary for various applications.

---

## Run the Sim!

Want to see the arm move before you build it? Run the sim! You can simulate the arm’s motion using MuJoCo too. Just follow the steps below.

### 1. Clone the repo (or download as zip)
```bash
git clone [https://github.com/devpatelcan/5dof.git]
cd 5-dof
```
> If cloning does not work, download as zip, extract files, and navigate to the file directory containing project folders (urdf, meshes, etc.) in your terminal.

### 2. Install dependencies
```bash
pip install mujoco
```

> **Requirements:** Python 3.9+. The `arm.xml` MuJoCo model and its `meshes/` folder must stay in the same directory as `run.py`, the script loads the XML relative to its own location, so cloning the repo as-is is all you need.

### 3. Run the sim
```bash
python run.py
```

This opens an interactive MuJoCo viewer with the arm loaded, and starts a prompt in your terminal.

**Controls / Usage:**
* Type 5 numbers separated by spaces - `X Y Z Pitch Roll` (meters and radians) - to send the arm to that pose, e.g. `0.200 0.050 0.150 0 0` (**Home**: 0 0 0 0 0)
* Type `R` to run a pre-programmed demo sequence that automatically visits 5 preset positions in a row
* Type `Q` to quit

If a target is out of the arm's physical reach, the terminal will print an error instead of crashing - just try a closer coordinate.

---

## Hardware Components

| Component | Function |
| :--- | :--- |
| **ESP32 Wrover Board** | Responsible for communication with FOC boards, servos, and telemetry with command interface.|
| **5010 BLDC Motor x2** | Base and shoulder joint actuation. |
| **DS3225 270**| Elbow joint actuation |
| **[PSU spec]** | Provides power to all electrical components. |
| **[Encoder model] x5** | Tracks each joint's angular position. |
| **[ESC/driver board]** | [Role] |

---

## Challenges and Fixes

| Issue | Resolution / Fix |
| :--- | :--- |
| Robotic arm would slam into table during testing, and would require manual shut-off. | Implemented automatic stall detection through the use of current sensing, position discrepancies, and thermistors. |
| Discovered a mismatch between coordinate system in arm firmaware, and real-life scenario. | Linked code to a simulation in MuJoCo to mimic simulated movement.  |
| Elbow servo moved far faster than base and shoulder actuators. | Designed an algorithm to calculate time it takes for cycloidal actuator to move to angle, and adjust the PWM duty cycle accordingly. |
| A method to control the arm through a custom GUI. | Designed sliders, stopping methods, and activation commands. Also designed to limit data going in and out FOC boards for robotic actuators. |

---

## More Pictures and Videos

<p align="center">
  <video src="[VIDEO_URL_1]" width="100%" autoplay loop muted playsinline></video>
</p>
[Caption describing what this video shows]

<br>

<p align="center">
  <video src="[VIDEO_URL_2]" width="100%" autoplay loop muted playsinline></video>
</p>
[Caption describing what this video shows]
