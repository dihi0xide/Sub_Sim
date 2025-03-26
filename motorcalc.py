import math as m

class motor:
    def __init__(self, angle, min_rpm, max_rpm, pos_x, pos_y, is_up_motor):
        self.angle = angle
        self.min_rpm = min_rpm
        self.max_rpm = max_rpm
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.is_up_motor = is_up_motor
    def start_motor(self, rpm):
        #this will be added later
        #we need to calculate a thrust force from rpm
        #for now this program calculates x and y thrust forces using the rpm as a base
        if self.min_rpm > rpm or rpm < self.max_rpm:
            raise Exception("The rpm specified was not with range of the of the motor limits")
        if self.is_up_motor == True:
            return[0, 0, rpm]
        else:
            return [(rpm * m.cos(self.angle)), (rpm * m.tan(self.angle)), 0]