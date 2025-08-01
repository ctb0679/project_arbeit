from urdfpy import URDF

robot = URDF.load('mycobot_urdf_fake.urdf')

for joint in robot.joints:
    print(f"Joint Name: {joint.name}")
    print(f"  Type: {joint.joint_type}")
    print(f"  Origin XYZ: {joint.origin[0]}")
    print(f"  Origin RPY: {joint.origin[1]}")
    print(f"  Axis: {joint.axis}\n")
