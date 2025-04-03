import math as m
import pybullet as p
import pybullet_data
import numpy as np
from time import sleep
import sub_sim.things

# Constants
WATER_DENSITY = 1000  # kg/m^3 (density of water)
GRAVITY = -9.81       # Gravity
DRAG_COEFFICIENT = 0.5  # Adjust for resistance
THRUST_FORCE = 50.0    # Strength of motors