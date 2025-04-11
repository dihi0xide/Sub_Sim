import pybullet as p
import pybullet_data

from sub_sim.vehicle import vehicle
from sub_sim.motor import motor

WATER_DENSITY = 1000  # kg/m^3 (density of water)
GRAVITY = -9.81       # Gravity
DRAG_COEFFICIENT = 0.5  # Adjust for resistance
THRUST_FORCE = 50.0    # Strength of motors

# Initialize PyBullet, the physics engine
physicsClient = p.connect(p.GUI)  # Display a GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY) # Setup gravity on the Z axis at -9.81 m/s

# Load ground for reference
p.loadURDF("plane.urdf")

print(vehicle)
sub = vehicle(10, 0.05, 0.1, [])
pos, orn = p.getBasePositionAndOrientation(sub.sub)
x, y, z = pos

#Side Motors
sub.motor_add(motor(-45, 0, 100, x + 0.1, y + 0.1, False))
sub.motor_add(motor(45, 0, 100, x - 0.1, y + 0.1, False))
sub.motor_add(motor(45, 0, 100, x - 0.1, y - 0.1, False))
sub.motor_add(motor(-45, 0, 100, x + 0.1, y - 0.1, False))

#Top Motors
sub.motor_add(motor(0, 0, 100, x + 0.1, y + 0.1, True))
sub.motor_add(motor(0, 0, 100, x - 0.1, y + 0.1, True))
sub.motor_add(motor(0, 0, 100, x - 0.1, y - 0.1, True))
sub.motor_add(motor(0, 0, 100, x + 0.1, y - 0.1, True))

while True:
    pos, orn = p.getBasePositionAndOrientation(sub.sub)
    x, y, z = pos
    p.stepSimulation()