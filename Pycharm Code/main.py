# This is the Drone Dynamics Python Script

# Starting from most basic items we will work from physical propellers to motors to frame and kinematics.
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

class Propeller:
    def __init__(self,diameter,pitch,efficiency):
        self.pitch=pitch # inches
        self.diameter=diameter
        self.efficiency=efficiency

    def dynamic_Thrust(self, RPM, airspeed,air_density):
        p1=1.225
        p2=3.1415926*pow(0.0254*self.diameter,2)*(1/4)
        p3=pow(RPM*0.0254*self.pitch*(1/60),2)
        p4=(RPM*0.0254*self.pitch*(1/60)*airspeed)
        p5=pow(self.diameter/(3.29546*self.pitch),1.5)
        return p1*p2*(p3-p4)*p5 #newtons

class Air:
    def __init__(self):
        self.density=1.225 #kg/m^3
        self.temp = 59 # C
        self.altitude= 0 # m above sea level
    def calc_density(self,alt):
        self.calc_Temp(alt)
        pressure=101.29*pow(((self.temp+273.1)/288.08),5.256) # kilopascal
        self.density=pressure/(0.2869*(self.temp+273.1)) # kg/m^3
        return self.density

    def calc_Temp(self,alt):
        self.temp=15.04-0.00649*alt # Celcius

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    prop=Propeller(10,6,0.9)
    print(prop.dynamic_Thrust(10600,0,1.225))
    env=Air()
    print(env.calc_density(0))


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
