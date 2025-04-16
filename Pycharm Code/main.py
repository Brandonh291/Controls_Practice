# This is the Drone Dynamics Python Script

# Starting from most basic items we will work from physical propellers to motors to frame and kinematics.


##############################################################
class Propeller:
    ################# Units #################
    # Pitch = Inches
    # Diameter = Inches
    # Efficiency = No Units
    # RPM = Rotations per Minute
    # Airspeed = Forward Airspeed in Meters per Second
    # Air Density = kilograms per cubic meter (kg/m^3)
    # Dynamic Thrust = Newtons
    
    def __init__(self,diameter,pitch,efficiency):
        self.pitch=pitch # inches
        self.diameter=diameter # Inches
        self.efficiency=efficiency # No units
        self.thrust=0
    def dynamic_Thrust(self, RPM, airspeed,air_density):
        p1=air_density
        p2=3.1415926*pow(0.0254*self.diameter,2)*(1/4)
        p3=pow(RPM*0.0254*self.pitch*(1/60),2)
        p4=(RPM*0.0254*self.pitch*(1/60)*airspeed) 
        p5=pow(self.diameter/(3.29546*self.pitch),1.5)
        self.thrust= p1*p2*(p3-p4)*p5 #newtons
        return self.thrust
##############################################################
class Air:
    ################# Units #################
    # Air Density = kilograms per cubic meter (kg/m^3)
    # Temperature = Celsius
    # Pressure = Kilopascal
    
    def __init__(self):
        self.density=1.225 #kg/m^3
        self.temp = 59 # C
        self.altitude= 0 # m above sea level
        
    def calc_density(self,alt):
        # This formula only covers the case of sea level up to 11,000 meters.
        self.calc_Temp(alt)
        pressure=101.29*pow(((self.temp+273.1)/288.08),5.256)
        self.density=pressure/(0.2869*(self.temp+273.1))
        return self.density

    def calc_Temp(self,alt):
        # This assumes that temperature will follow a simple curve from roughly 59 degF at sea level up to 11,000 meters.
        self.temp=15.04-0.00649*alt # Celcius


class Drone:

    def __init__(self):
        self.mass=1 # kg


class Frame:

    def __init__(self):
        self.roll=0
        self.pitch=0
        self.yaw=0
        self.x=0
        self.y=0
        self.z=0

    def Drone_to_Global(self,roll,pitch,yaw):
        #roll_matrix = [1,    0,            0;
        #               0,    Cos(roll),    -Sin(roll);
        #               0,    Sin(roll),    Cos(roll);]

        #pitch_matrix = [cos(pitch),  0,    sin(pitch);
        #                0,           1,    0;
        #                -sin(pitch),    0,    Cos(pitch);]

        #yaw_matrix =  [cos(yaw),    -sin(yaw),    0;
        #               sin(yaw),    Cos(roll),    0;
        #               0,                   0,    1;]

        # Rotation_Matrix = yaw*pitch*roll
        # Global_Frame=Rotation_Matrix*Inertial_Frame




class PointMass:
    # This wasa test for a simple model for a point mass with thrust pointing up.
    def __init__(self):
        self.speed=0 # meters/second
        self.position=0 # meters
        self.mass=1 # kg
        self.accel=0 #m/s^2
    def calc_vel(self,thrust,timestep):
        self.accel=(thrust-9.81)/self.mass
        self.speed = self.speed+self.accel*timestep
        return self.speed

    def calc_pos(self,timestep):
        self.position = self.position + self.speed*timestep
        return self.position
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    prop=Propeller(10,6,0.9)
    env=Air()
    car=PointMass()
    timestep=1
    for i in range(0,3000,1):
        density=env.calc_density(car.position)
        car.calc_vel(prop.dynamic_Thrust(10000,car.speed,density),timestep)
        car.calc_pos(timestep)
        time=i*timestep
        print(time,',',car.position,",",car.speed,",",car.accel,",",prop.thrust,",",env.density)
