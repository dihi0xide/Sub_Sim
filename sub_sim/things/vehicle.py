import math as m
import pybullet as p
import pybullet_data
import numpy as np
from time import sleep

class vehicle:
    def __init__(self, mass, volume, cross_sectional_area, motors):
        self.mass = mass
        self.volume = volume
        self.cross_sectional_area = cross_sectional_area
        self.motors = motors
        self.sub = p.loadURDF("cube_small.urdf", [0, 0, -1], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=2.0)
        if not isinstance(self.motors, list):
            raise Exception("Motors is not a list")
    def motor_add(self, motor):
        if isinstance(motor, list) == True:
            raise Exception("motor_add only accepts motors")
        else:
            self.motors.add = motor
    def step(self, tick):
        pos, orn = p.getBasePositionAndOrientation(sub)
        vel, ang_vel = p.getBaseVelocity(sub)
        if pos[2] < 0:
            buoyancy_force = WATER_DENSITY * self.volume * 9.81
            p.applyExternalForce(sub, -1, [0, 0, buoyancy_force], pos, p.WORLD_FRAME)
        drag_force = -DRAG_COEFFICIENT * np.array(vel) * nm.linalg.norm(vel) * self.cross_sectional_area
        for i in len(self.motor):
            p.applyExternalForce(sub, -1, motor.start_motor(), [motor.pos_x, motor.pos_y, 0], p.WORLD_FRAME)
        p.stepSimulation()
        sleep(tick)