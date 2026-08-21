import mujoco

# Use the absolute path so Python finds it regardless of where the terminal is open
urdf_path = r"C:\Dev's Stuff\Personal Projects\5DOF\Robotic_Sim\New_urdf\urdf\New_urdf.urdf"

# Read the URDF and save it as a native MuJoCo XML
model = mujoco.MjModel.from_xml_path(urdf_path)
mujoco.mj_saveLastXML("Robotic_Sim.xml", model)