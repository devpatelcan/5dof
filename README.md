# 5-DOF Robotic Arm

<img width="562.5" height="750" alt="20260819_220313" src="https://github.com/user-attachments/assets/e7e38184-a4f2-48fd-8ef7-fed7db602751" />

I built a custom 5-degree-of-freedom robotic arm, driven by two 20:1 cycloidal drives and 3 servos. The system achieves a 440mm reach, with accurate inverse kinematics (IK) and trajectory planning. This setup delivers the reach and ease of use necessary for various applications.

---

## Run the Sim!

Want to see the arm move before you build it? Run the sim! You can simulate the arm’s motion using MuJoCo too. Just follow the steps below.

### 1. Clone the repo (or download as a zip)
```bash
git clone [[https://github.com/devpatelcan/5dof.git](https://github.com/devpatelcan/5dof.git)]
cd 5-dof
```
> If cloning does not work, download as a zip, extract the files, and navigate to the file directory containing the project folders (urdf, meshes, etc.) in your terminal.

### 2. Install dependencies
```bash
pip install mujoco
```

> **Requirements:** Python 3.9+. The `arm.xml` MuJoCo model and its `meshes/` folder must stay in the same directory as `run.py`. The script loads the XML relative to its own location, so cloning the repo as-is is all you need.

### 3. Run the sim
```bash
python run.py
```

This opens an interactive MuJoCo viewer with the arm loaded and starts a prompt in your terminal.
> [!NOTE]
> **Fun Fact: MuJoCo was used to verify the IK solver, joint limits, and motor synchronization. If the arm were made of isotropic materials (e.g., metals), MuJoCo could verify the arm's performance under load and gravity!** 

**Controls / Usage:**
* Type 5 numbers separated by spaces - `X Y Z Pitch Roll` (meters and radians) - to send the arm to that pose, e.g. `0.200 0.050 0.150 0 0` (**Home**: 0 0 0 0 0)
* Type `R` to run a pre-programmed demo sequence that automatically visits 5 preset positions in a row
* Type `Q` to quit

If a target is out of the arm's physical reach, the terminal will print an error instead of crashing - just try a closer coordinate.

---

## Hardware Components

| Component | Function |
| :--- | :--- |
| **ESP32 Wrover Board** | Responsible for communication with FOC boards, servos, and telemetry via a command interface.|
| **5010 BLDC Motor x2** | Base and shoulder joint actuation. |
| **DS3225 270**| Elbow joint actuation. |
| **MG996R Servos**| Wrist differential actuation. |
| **12V 20A PSU** | Provides power to all electrical components. |
| **MT6816** | Tracks each joint's angular position via SPI. |
| **SimpleFOC Board x2** | Controlled BLDC actuators for base and shoulder. |

---

## Challenges and Fixes

| Issue | Resolution / Fix |
| :--- | :--- |
| The robotic arm would slam into the table during testing and would require manual shut-off. | Implemented automatic stall detection through the use of current sensing, position discrepancies, and thermistors. |
| Discovered a mismatch between the coordinate system in the arm's firmware and the real-life scenario. | Linked the code to a simulation in MuJoCo to mimic real-world movement. |
| The elbow servo moved far faster than the base and shoulder actuators. | Designed an algorithm to calculate the time it takes for the cycloidal actuator to move to a given angle and adjusted the PWM duty cycle accordingly. |
| Needed a method to control the arm through a custom GUI. | Designed sliders, stopping methods, and activation commands. Also implemented limits on data going in and out of the FOC boards for the robotic actuators. |

---

## More Pictures and Videos

https://github.com/user-attachments/assets/3cee493c-4cad-49be-bfc4-cf6142092e35

*MuJoCo simulation of the arm.*



<img width="562.5" height="750" alt="20260819_215127" src="https://github.com/user-attachments/assets/3a8cbc28-5555-439c-98db-ff00764c0456" />
<img width="562.5" height="750" alt="20260819_214749" src="https://github.com/user-attachments/assets/07f362be-6022-4bdf-ba88-9948b7f4abd6" />

*Arm in different positions.*

https://github.com/user-attachments/assets/0d206a8f-8fe6-4985-8a2d-9268c4d23881

*Testing the BLDC actuator with an MT6816 (an encoder that supports SPI communication).*

> [!NOTE]
> **Fun Fact: Since the MT6816 communicates via SPI, it is much FASTER than the AS5600, which uses I2C. The MT6816 is approximately 12x faster than the AS5600 encoder!** 


https://github.com/user-attachments/assets/77c50f5f-8f70-4125-b3a2-351496e4a2c5

*Initial startup test of the arm. The arm is operated through a custom GUI.*
