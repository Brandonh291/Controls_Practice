# -*- coding: utf-8 -*-
# This is the Drone Dynamics Python Script

# Starting from most basic items we will work from physical propellers to motors to frame and kinematics.


# roll_matrix = [1,    0,            0;
#               0,    Cos(roll),    -Sin(roll);
#               0,    Sin(roll),    Cos(roll);]

# pitch_matrix = [cos(pitch),  0,    sin(pitch);
#                0,           1,    0;
#                -sin(pitch),    0,    Cos(pitch);]

# yaw_matrix =  [cos(yaw),    -sin(yaw),    0;
#               sin(yaw),    Cos(roll),    0;
#               0,                   0,    1;]

# Rotation_Matrix = yaw*pitch*roll
# Global_Frame=Rotation_Matrix*Inertial_Frame

##############################################################
import random
import matplotlib.pyplot as plt
import numpy as np


class PID:
    def __init__(self, P, I, D):
        self.P = 0
        self.I = 0
        self.D = 0


##############################################################
class Motor_Propeller:
    ################# Units #################
    # Pitch = Inches
    # Diameter = Inches
    # Efficiency = No Units
    # RPM = Rotations per Minute
    # Airspeed = Forward Airspeed in Meters per Second
    # Air Density = kilograms per cubic meter (kg/m^3)
    # Dynamic Thrust = Newtons
    # Noise is a % error applied to the input throttle when calculating thrust

    def __init__(self, diameter, pitch, efficiency, MaxVolt, MaxPower, Kv, noise):
        self.pitch = pitch  # inches
        self.diameter = diameter  # Inches
        self.efficiency = efficiency  # No units
        self.thrust = 0  # Newtons
        self.Kv = Kv  # RPM per volt
        self.NomVolt = MaxVolt  # Nominal Voltage of Power Source
        self.MaxPower = MaxPower  # Watts
        self.curPower = 0  # Watts
        self.RPM = 0  # RPM
        self.torque = 0  #
        self.noise = noise  # %

    def calc_MotorParam(self, throttle):
        self.RPM = self.Kv * self.NomVolt * (throttle / 100)
        self.curPower = self.MaxPower * (throttle / 100)

    def dynamic_Thrust(self, throttle, airspeed, air_density):
        # This will generate a random throttle within +- noise% of the current.
        # So at 50% throttle we have a range of 49.5 to 50.5
        if self.noise == 0:
            noisy_throttle = throttle
        else:
            noisy_throttle = random.randrange((throttle * (100 - self.noise)), (throttle * (100 + self.noise)), 1) / 100

        self.calc_MotorParam(noisy_throttle)
        p1 = air_density
        p2 = 3.1415926 * pow(0.0254 * self.diameter, 2) * (1 / 4)
        p3 = pow(self.RPM * 0.0254 * self.pitch * (1 / 60), 2)
        p4 = (self.RPM * 0.0254 * self.pitch * (1 / 60) * airspeed)
        p5 = pow(self.diameter / (3.29546 * self.pitch), 1.5)
        self.thrust = p1 * p2 * (p3 - p4) * p5  # newtons
        if self.RPM > 0:
            self.torque = (60 * self.curPower) / (2 * 3.1415 * self.RPM)
        else:
            self.torque = 0

        return self.thrust


##############################################################
class Air:
    ################# Units #################
    # Air Density = kilograms per cubic meter (kg/m^3)
    # Temperature = Celsius
    # Pressure = Kilopascal

    def __init__(self):
        self.density = 1.225  # kg/m^3
        self.temp = 59  # C
        self.altitude = 0  # m above sea level

    def calc_density(self, alt):
        # This formula only covers the case of sea level up to 11,000 meters.
        self.calc_Temp(alt)
        pressure = 101.29 * pow(((self.temp + 273.1) / 288.08), 5.256)
        self.density = pressure / (0.2869 * (self.temp + 273.1))
        return self.density

    def calc_Temp(self, alt):
        # This assumes that temperature will follow a simple curve from roughly 59 degF at sea level up to 11,000 meters.
        self.temp = 15.04 - 0.00649 * alt  # Celcius


##############################################################


class Drone:
    def __init__(self, Ixx, Iyy, Izz, L):
        self.L = L  # Distance from center of quadcopter to propeller

        # Motors
        self.mot_1 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 5)
        self.mot_2 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 5)
        self.mot_3 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 5)
        self.mot_4 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 5)

        # Position
        self.Pos_X = 0  # meters
        self.Pos_Y = 0  # meters
        self.pos_Z = 0  # meters

        # Velocity
        self.Vel_X = 0  # meters/second
        self.Vel_Y = 0  # meters/second
        self.Vel_Z = 0  # meters/second

        # Acceleration
        self.Acc_X = 0  # meters/second^2
        self.Acc_Y = 0  # meters/second^2
        self.Acc_Z = 0  # meters/second^2

        # Angle
        self.roll = 0  # degrees
        self.pitch = 0  # degrees
        self.yaw = 0  # degrees

        # Angular Rate
        self.roll_rate = 0  # degrees/second
        self.pitch_rate = 0  # degrees/second
        self.yaw_rate = 0  # degrees/second

        # Angular Acceleration
        self.roll_acc = 0  # degrees/second^2
        self.pitch_acc = 0  # degrees/second^2
        self.yaw_acc = 0  # degrees/second^2

        # Physical Items
        self.mass = 1  # kg

        # Controller outputs (refers to throttle %)
        self.cont_1 = 0
        self.cont_2 = 0
        self.cont_3 = 0
        self.cont_4 = 0

    def controller(self, des_x, des_y, des_z):

    def updateFrame(self, timestep, air_density, des_x, des_y, des_z):
        self.controller(des_x, des_y, des_z)
        # Linear Motion
        TotalThrust = self.mot_1.dynamic_thrust(self.cont_1, Vel_Z, air_density) + self.mot_2.dynamic_thrust(
            self.cont_2, Vel_Z, air_density) + self.mot_3.dynamic_thrust(self.cont_3, Vel_Z,
                                                                         air_density) + self.mot_4.dynamic_thrust(
            self.cont_4, Vel_Z, air_density)

    def calc_vel(self, thrust, timestep):
        self.accel = (thrust - 9.81) / self.mass
        self.speed = self.speed + self.accel * timestep
        return self.speed

    def calc_pos(self, timestep):
        self.position = self.position + self.speed * timestep
        return self.position


##############################################################

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    x = []
    y = []
    prop = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 0)
    drone = Drone(1, 1, 1, 10)
    env = Air()

    airspeed = 0
    for alt in range(0, 11000, 500):
        density = env.calc_density(alt)
        thrust = prop.dynamic_Thrust(50, airspeed, density)
        # print(alt)
        print(thrust, ",", prop.torque)
        x.append(alt)
        y.append(thrust)

    plt.plot(x, y)
    plt.show


