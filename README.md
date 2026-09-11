# 5-DOF Robotic Arm
<a href="[YOUTUBE_LINK]">
  <img src="[YOUTUBE_THUMBNAIL_URL]" width="600">
</a>

[Click here or on the image to watch the video]([YOUTUBE_LINK])




I built a custom 5-degree-of-freedom robotic arm, driven by 2 20:1 cycloidal drives and 3 servos. The system achieves a 440mm reach, wih accurate inverse kinematics (IK) and trajectory planning. This setup delivers the reach and ease of use necessary for research applications.

---

## Try the Simulation

Want to see the arm move before you build it? Run the sim! You can simulate the arms motion using MuJoCo too. Just follow the steps below.

### 1. Clone the repo (or download as zip)
```bash
git clone [https://github.com/devpatelcan/5dof.git]
cd 5-dof
```

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
| **DS3225 270**|| Elbow joint actuation |
| **[PSU spec]** | Provides power to all electrical components. |
| **[Encoder model] x5** | Tracks each joint's angular position. |
| **[ESC/driver board]** | [Role] |

---

## Challenges and Fixes

| Issue | Resolution / Fix |
| :--- | :--- |
| [Issue 1] | [Fix 1] |
| [Issue 2] | [Fix 2] |
| [Issue 3] | [Fix 3] |
| [Issue 4] | [Fix 4] |

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
