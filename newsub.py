import sub_sim

# Initialize PyBullet, the physics engine
physicsClient = p.connect(p.GUI)  # Display a GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY) # Setup gravity on the Z axis at -9.81 m/s

# Load ground for reference
p.loadURDF("plane.urdf")

print(vehicle)
sub = vehicle.vehicle(10, 0.05, 0.1, [])
pos, orn = p.getBasePositionAndOrientation(sub.sub)
x, y, z = pos

#Side Motors
sub.motor_add(a = sub_sim.motor(-45, 0, 100, x + 0.1, y + 0.1, False))
sub.motor_add(b = motor(45, 0, 100, x - 0.1, y + 0.1, False))
sub.motor_add(c = motor(45, 0, 100, x - 0.1, y - 0.1, False))
sub.motor_add(d = motor(-45, 0, 100, x + 0.1, y - 0.1, False))

#Top Motors
sub.motor_add(e = motor(0, 0, 100, x + 0.1, y + 0.1, True))
sub.motor_add(f = motor(0, 0, 100, x - 0.1, y + 0.1, True))
sub.motor_add(g = motor(0, 0, 100, x - 0.1, y - 0.1, True))
sub.motor_add(h = motor(0, 0, 100, x + 0.1, y - 0.1, True))

for _ in range(10000):
    pos, orn = p.getBasePositionAndOrientation(sub.sub)
    x, y, z = pos