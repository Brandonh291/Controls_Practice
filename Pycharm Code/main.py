# This is the Drone Dynamics Python Script

# Starting from most basic items we will work from physical propellers to motors to frame and kinematics.
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

class Propeller:
    def __init__(self,diameter,pitch,efficiency):
        self.pitch=pitch
        self.diameter=diameter
        self.efficiency=efficiency

    def dynamic_Thrust(self, RPM, airspeed,air_density):
        p1=1.225
        p2=3.1415926*pow(0.0254*self.diameter,2)*(1/4)
        p3=pow(RPM*0.0254*self.pitch*(1/60),2)
        p4=(RPM*0.0254*self.pitch*(1/60)*airspeed)
        p5=pow(self.diameter/(3.29546*self.pitch),1.5)
        return p1*p2*(p3-p4)*p5



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    prop=Propeller(10,6,0.9)
    print(prop.dynamic_Thrust(10600,0,1.225))

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
