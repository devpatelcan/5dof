import mujoco
import mujoco.viewer
import math
import sys
import time
import threading
import os

# ============================================================
#  5-DOF Arm — Interactive Inverse Kinematics (IK) Tester
# ============================================================
# This script loads the MuJoCo simulation, lets you type a target
# end-effector pose (X, Y, Z, Pitch, Roll) into the terminal, solves
# for the joint angles that reach that pose, and smoothly animates
# the arm to it in the viewer window.
#
# Type "R" instead of coordinates to run a pre-programmed sequence
# that automatically visits 5 preset positions in a row.
# See reg_seq below.
# ============================================================

# Custom IK solver ⌄⌄⌄

# Arm Constants (Meters)
# L1 / L2 = link lengths of the upper arm (shoulder->elbow) and
#           forearm (elbow->wrist), used in the 2-link IK triangle.
# H0      = vertical height of the shoulder joint above the base origin.
# D0      = horizontal offset of the shoulder joint from the base's
#           vertical rotation axis.
L1 = 0.220
L2 = 0.220
H0 = 0.08964862567
D0 = 0.02185974963



def calculate_analytical_ik(x, y, z, pitch, roll):

    #Calculate the required joint angles to reach a target coordinate.
    #Input: Target X, Y, Z (meters) and Pitch, Roll (radians).
    #Output: Measured joint angles, while maintaining eblow always above table.
    
    # ----- Origin offset ------
    
    # An offset applied to match the end efector (EE) origin with the MuJoCo global origin
    x_offset = 0.10  
    y_offset = 0.00
    z_offset = 0.07  
    
    # Shift the user's requested target by the offset above so that
    # (0, 0, 0) typed by the user lines up with the arm's real-world
    # "home" reference point instead of MuJoCo's internal origin.
    x_target = x + x_offset
    y_target = y + y_offset
    z_target = z + z_offset



    # --- Kinematic Calculations ---
    # 1. Base Angle
    # Rotates the whole arm left/right to point at the target (viewed
    # from directly above, looking down the Z axis).
    theta1 = math.atan2(y_target, x_target)

    # 2. 2D Projection (Project 3D arm in space onto a 2D XY plane)
    # Once the base is aimed at the target, the shoulder/elbow only
    # need to solve a flat 2D problem: "reach distance" (r_planar)
    # vs. "height" (z_planar), like a 2-link arm swinging in a vertical plane.
    R = math.sqrt(x_target**2 + y_target**2)
    r_planar = R - D0
    z_planar = z_target - H0

    # Total distance from shoulder joint to wrist joint
    d = math.sqrt(r_planar**2 + z_planar**2)

   
    # Print an error message if position is mathematically not solvable
    # (target is further away than the arm's two links can physically reach)
    if d > (L1 + L2):
        print("\n[!] ERROR: Coordinate out of physical reach! Try closer coordinates.\n")
        return None

    # 3. Elbow Angle (Cosine Law)
    # Standard 2-link IK: solve the triangle formed by L1, L2, and d
    # (the straight-line distance to the target) for the elbow bend angle.
    cos_theta3 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))  # clamp to avoid math domain errors from float rounding

    # Forces a negative angle to ensure the elbow never goes beneath the table 
    theta3 = -math.acos(cos_theta3)  

    # 4. Shoulder Angle
    # Adds beta to alpha to maintain elbow above the table
    # alpha = angle from the shoulder straight to the target
    # beta  = extra angle needed to "bend" toward the elbow-up solution
    alpha = math.atan2(z_planar, r_planar)
    beta = math.acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
    theta2 = alpha + beta 

    # 5. Wrist diff
    # The wrist is a differential (2 motors driving pitch + roll together),
    # so the desired pitch/roll are combined into two opposing motor commands.
    left_wrist_angle = pitch + roll
    right_wrist_angle = -pitch + roll

    return {
        "base": theta1,
        "shoulder": theta2,
        "elbow": theta3,
        "left_diff": left_wrist_angle,
        "right_diff": right_wrist_angle,
        "pitch": pitch,
        "roll": roll
    }


# MuJoCo Simulation Environment ⌄⌄⌄

# Max Joint speed & FPS
MAX_SPEED_DEG_PER_SEC = 90.0  # caps how fast any single joint animates, so motion looks smooth instead of teleporting
FPS = 60.0 

# Load .xml file
# Robotic_Sim.xml is expected to live in the same folder as this script.
# We build the path off the script's own location on disk (instead of a
# hardcoded absolute path or a path relative to the current working
# directory), so this works no matter where the script is run from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(SCRIPT_DIR, "arm.xml")

try:
    model = mujoco.MjModel.from_xml_path(model_path)
except ValueError:
    print(f"Error: Could not find arm.xml at {model_path}. Make sure it's in the same folder as this script.")
    sys.exit()

# Disable gravity (keep physics enabled)
model.opt.gravity[:] = [0, 0, 0]
data = mujoco.MjData(model)

# Map each joint's name (as defined in the MJCF/URDF) to its MuJoCo joint ID,
# so we can read/write that joint's position by index later on.
joint_names = {
    "base": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm Base"),
    "shoulder": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_shoulder"),
    "elbow": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_elbow"),
    "left_wrist": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_leftwrist"),
    "right_wrist": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_Rightwrist"),
    "wrist_pitch": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_wristoutput"),
    "wrist_roll": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm_wristroll")
}

# Ordered list of qpos (joint position) array indices, in the same order
# the IK solver returns its 7 output angles, so we can copy angles straight
# from calculate_analytical_ik()'s output into the right slot in data.qpos.
joint_ids = [
    model.jnt_qposadr[joint_names["base"]],
    model.jnt_qposadr[joint_names["shoulder"]],
    model.jnt_qposadr[joint_names["elbow"]],
    model.jnt_qposadr[joint_names["left_wrist"]],
    model.jnt_qposadr[joint_names["right_wrist"]],
    model.jnt_qposadr[joint_names["wrist_pitch"]],
    model.jnt_qposadr[joint_names["wrist_roll"]]
]


# Demo: a canned list of (X, Y, Z, Pitch, Roll) poses
# that the arm will visit one after another, pausing 2 seconds at each
# stop, whenever the user types 'R' at the terminal prompt instead of
# a coordinate. 

reg_seq = [
    (0.00,  0.00, 0.00,  0.00,  0.00),
    (0.14, -0.04, 0.18, -0.77,  0.77),
    (-0.07, 0.11, 0.02, -0.74,  0.74),
    (-0.07, 0.09, 0.21,  2.00,  2.00),
    (0.11,  0.09, 0.03, -2.00, -2.00),
    (0.00,  0.00, 0.00,  0.00,  0.00),
]
reg_seq_GAP_SEC = 2.0


def move_to_target(viewer, target_x, target_y, target_z, target_pitch, target_roll):
    """Compute IK for the given target and smoothly animate the arm to it."""

    angles = calculate_analytical_ik(target_x, target_y, target_z, target_pitch, target_roll)

    if not angles:
        return

    print(f"Angles calculated >> Base: {math.degrees(angles['base']):.1f}°, Shoulder: {math.degrees(angles['shoulder']):.1f}°, Elbow: {math.degrees(angles['elbow']):.1f}°")

    # Apply custom MuJoCo Calibrations
    mujoco_shoulder = 1.79 - angles["shoulder"]
    mujoco_elbow = -1.26 - angles["elbow"]

    target_qpos = [
        angles["base"],
        mujoco_shoulder,
        mujoco_elbow,
        angles["left_diff"],
        angles["right_diff"],
        angles["pitch"],
        angles["roll"]
    ]

    current_qpos = [data.qpos[i] for i in joint_ids]  # Current arm joint positions

    # Linear animation for arm motion.
    # Find whichever joint has to travel the furthest, and use its
    # travel time (at MAX_SPEED_DEG_PER_SEC) to set how long the whole
    # move takes -- this way every joint arrives at the same moment
    # instead of some finishing early and "snapping" into place.
    max_diff_rad = max(abs(t - c) for t, c in zip(target_qpos, current_qpos))
    max_speed_rad = math.radians(MAX_SPEED_DEG_PER_SEC)

    if max_diff_rad > 0:
        duration = max_diff_rad / max_speed_rad
        total_steps = max(1, int(duration * FPS))

        # Step every joint a little closer to its target each frame
        # (linear interpolation) until all 7 joints reach target_qpos.
        for step in range(1, total_steps + 1):
            fraction = step / total_steps

            for i in range(7):
                step_pos = current_qpos[i] + (target_qpos[i] - current_qpos[i]) * fraction
                data.qpos[joint_ids[i]] = step_pos

            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            viewer.sync()

            time.sleep(1.0 / FPS)


def run_reg_seq(viewer):
    print("\n[*] Running regular sequence...\n")
    for i, (sx, sy, sz, spitch, sroll) in enumerate(reg_seq, start=1):
        print(f"[*] sequence step {i}/{len(reg_seq)}: {sx} {sy} {sz} {spitch} {sroll}")
        move_to_target(viewer, sx, sy, sz, spitch, sroll)
        time.sleep(reg_seq_GAP_SEC)
    print("\n[*] regular sequence complete.\n")


# Terminal Listener
user_input_command = None

def terminal_listener():
    """
    Runs on a background thread and blocks on input() so the viewer/physics
    loop below can keep rendering while we wait for the user to type something.
    Whatever the user types gets stored in the global `user_input_command`,
    and the main loop picks it up and acts on it.
    """
    global user_input_command
    print("="*50)
    print("IK Tester started")
    print("Format: X Y Z Pitch Roll")
    print("Units: Meters and Radians")
    print("Ex, 0.200 0.050 0.150 0 0")
    print("-"*50)
    print("Special commands:")
    print("  R  -> run the regular 5-position demo sequence")
    print("  Q  -> quit the program")
    print("="*50)
    
    while True:
        cmd = input("\nEnter coords (X Y Z Pitch Roll), or R for the demo sequence, or Q to quit: ")
        user_input_command = cmd
        if cmd.lower().strip() == 'q':
            os._exit(0)

# Launch the interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    # Start listener
    threading.Thread(target=terminal_listener, daemon=True).start()
    
    while viewer.is_running():
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        if user_input_command is not None:
            current_input = user_input_command
            user_input_command = None 

            # regular seq trigger: typing "R" (case-insensitive) skips
            # normal coordinate parsing entirely and runs the 5-stop demo.
            if current_input.strip().upper() == 'R':
                run_reg_seq(viewer)
                continue
            
            # Otherwise, expect exactly 5 space-separated numbers:
            # X Y Z Pitch Roll (meters, meters, meters, radians, radians)
            try:
                vals = [float(v) for v in current_input.strip().split()]
                if len(vals) != 5:
                    print("[!] Error: You must enter exactly 5 numbers separated by spaces.")
                    continue
                    
                target_x, target_y, target_z, target_pitch, target_roll = vals
                
            except ValueError:
                print("[!] Error: Invalid input. Please enter numbers only.")
                continue
                
            # Do math and animate
            move_to_target(viewer, target_x, target_y, target_z, target_pitch, target_roll)
                        
        time.sleep(1.0 / FPS)