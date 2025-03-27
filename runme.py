import pybullet as p
import pybullet_data
import time
import numpy as np
import math as m
from motorcalc import motor

# Constants
WATER_DENSITY = 1000  # kg/m^3 (density of water)
GRAVITY = -9.81       # Gravity
DRAG_COEFFICIENT = 0.5  # Adjust for resistance
THRUST_FORCE = 50.0    # Strength of motors

# Motor Pos
# MU1 = (1,1)
# MU2 = (1,-1)
# MU3 = (-1,-1)
# MU4 = (-1,1)
# M1 = (2,2)
# M2 = (2,-2)
# M3 = (-2,-2)
# M4 = (-2,2)


# Initialize PyBullet, the physics engine
physicsClient = p.connect(p.GUI)  # Display a GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY) # Setup gravity on the Z axis at -9.81 m/s

# Load ground for reference
p.loadURDF("plane.urdf")

# Create underwater object (submarine)
start_pos = [0, 0, -1]  # Start submerged in water
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
box_id = p.loadURDF("cube_small.urdf", start_pos, start_orientation, globalScaling=2.0)

# Object properties, self explanitory
mass = 10.0  # kg
volume = 0.05  # m³ (approximate)
cross_sectional_area = 0.1  # Used for drag calculations

# Simulation loop
for _ in range(10000):
    time.sleep(1 / 240)  # PyBullet runs at 240Hz by default (tick rate)

    # Get object's position and velocity, i think these are tuples
    pos, orn = p.getBasePositionAndOrientation(box_id)
    # print(p.getBasePositionAndOrientation(box_id))
    vel, ang_vel = p.getBaseVelocity(box_id)

    # BUOYANCY: Apply upward force if submerged
    if pos[2] < 0:  # Below water surface, surface = 2
        buoyancy_force = WATER_DENSITY * volume * -GRAVITY  # Archimedes' Principle
        p.applyExternalForce(box_id, -1, [0, 0, buoyancy_force], pos, p.WORLD_FRAME)

    # DRAG: Apply resistance to movement
    drag_force = -DRAG_COEFFICIENT * np.array(vel) * np.linalg.norm(vel) * cross_sectional_area
    p.applyExternalForce(box_id, -1, drag_force.tolist(), pos, p.WORLD_FRAME)

    # Keyboard listener
    keys = p.getKeyboardEvents()
    # print(keys) # this was for debug things dw about it
    # Seperate pos into x, y, and z
    x, y, z = pos
    # Position of the motors reletive to the sub
    pos_MU1 = (x + 1, y + 1, z)
    pos_MU2 = (x + 1, y - 1, z)
    pos_MU3 = (x - 1, y - 1, z)
    pos_MU4 = (x - 1, y + 1, z)
    pos_M1 = (x + 2, y + 2, z)
    pos_M2 = (x + 2, y - 2, z)
    pos_M3 = (x - 2, y - 2, z)
    pos_M4 = (x - 2, y + 2, z)
    # Cap the z-axis
    if z > 5:
        p.resetBasePositionAndOrientation(box_id, [x, y, 5], orn)
    # Print the position of the sub
    print(pos)
    # p.applyExternalForce: creates a force (motors), Documentation here: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.mq73m9o2gcpy
    if 65297 in keys:  # Move forward
        p.applyExternalForce(box_id, -1, [0, 0, THRUST_FORCE], pos_MU1, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [0, 0, THRUST_FORCE], pos_MU2, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [0, 0, THRUST_FORCE], pos_MU3, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [0, 0, THRUST_FORCE], pos_MU4, p.WORLD_FRAME)
    if 65298 in keys:  # Move backward
        p.applyExternalForce(box_id, -1, [0, 0, -THRUST_FORCE], pos_MU1, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [0, 0, -THRUST_FORCE], pos_MU2, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [0, 0, -THRUST_FORCE], pos_MU3, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [0, 0, -THRUST_FORCE], pos_MU4, p.WORLD_FRAME)
    if 65295 in keys:  # Turn left
        p.applyExternalForce(box_id, -1, [(THRUST_FORCE * m.cos(40)), (THRUST_FORCE * m.tan(40)), 0], pos_M1, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [(THRUST_FORCE * m.cos(-40)), (THRUST_FORCE * m.tan(-40)), 0], pos_M2, p.WORLD_FRAME)
    if 65296 in keys:  # Turn right
        p.applyExternalForce(box_id, -1, [(-THRUST_FORCE * m.cos(40)), (-THRUST_FORCE * m.tan(40)), 0], pos_M1, p.WORLD_FRAME)
        p.applyExternalForce(box_id, -1, [(-THRUST_FORCE * m.cos(-40)), (-THRUST_FORCE * m.tan(-40)), 0], pos_M2, p.WORLD_FRAME)

    # Step simulation (+1 tick)
    p.stepSimulation()

# Disconnect after finishing
p.disconnect()