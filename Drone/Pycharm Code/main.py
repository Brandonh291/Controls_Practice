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
import math


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
        elif throttle == 0:
            noisy_throttle = 0
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
    def __init__(self, Ixx, Iyy, Izz, L, Mass):
        # Distance from Center of Quad to center of motor
        self.L = L  # Meters

        # Motors
        self.mot_1 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 0)
        self.mot_2 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 0)
        self.mot_3 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 0)
        self.mot_4 = Motor_Propeller(10, 6, 0.9, 12, 100, 2000, 0)
        self.Motors = [self.mot_1, self.mot_2, self.mot_3, self.mot_4]
        # Environment
        self.air = Air()

        # Position
        self.Lin_Pos = [0, 0, 0]  # meters

        # Velocity
        self.Lin_Vel = [0, 0, 0]  # meters/second

        # Acceleration
        self.Lin_Acc = [0, 0, 0]  # meters/second^2

        # Angle
        self.Ang_Pos = [0, 0, 0]  # Degrees

        # Angular Rate
        self.Ang_Vel = [0, 0, 0]  # Deg/Sec

        # Angular Acceleration
        self.Ang_Acc = [0, 0, 0]  # Deg/Sec^2

        # Physical Items
        self.mass = Mass  # kg

        # Controller outputs (refers to throttle %)
        self.PID_Out = [0, 0, 0, 0]  # %

        # Moment of Inertia
        self.MoI = [Ixx, Iyy, Izz]  # kg/m^2

        # Gravity
        self.g = 9.81  # m/s^2

        # Rotor Speed
        self.Omega = [0, 0, 0, 0]  # rad/s

        # Thrust
        self.F_total = 0  # newtons
        self.F_mot = [0, 0, 0, 0]  # newtons

        # Control Torque
        self.Tau = [0, 0, 0]  # Newton Meters

    def calc_state(self):
        # Air Density
        airDensity = self.air.calc_density(self.Lin_Pos[2])
        print(airDensity)

        # Thrust
        self.F_total = 0
        for i in range(len(self.Motors)):
            self.F_mot[i] = self.Motors[i].dynamic_Thrust(self.PID_Out[i], self.Lin_Vel[2], airDensity)
            self.F_total = self.F_total + self.F_mot[i]
            print("Thrust of Motor " + str(i) + ":" + str(self.F_mot[i]))
        print("Total For all Motors:" + str(self.F_total))

        # Torque
        self.Tau[0] = self.L * (self.Motors[2].torque - self.Motors[0].torque)
        self.Tau[1] = self.L * (self.Motors[3].torque - self.Motors[1].torque)
        self.Tau[2] = (self.Motors[1].torque + self.Motors[3].torque - self.Motors[0].torque - self.Motors[2].torque)
        print("Torque" + str(self.Tau))

        self.Lin_Vel[0]
        self.Lin_Vel[1]
        self.Lin_Vel[2]

        self.Lin_Pos[0]
        self.Lin_Pos[1]
        self.Lin_Pos[2]


##############################################################

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    x = []
    y = []
    drone = Drone(1, 1, 1, 10, 1)
    drone.calc_motor_thrust()

    plt.plot(x, y)
    plt.show


