import math
import matplotlib.pyplot as plt

class Wind:
    def __init__(self, z_c=200, direction=0, reference_wind=20):
        self.z_c = z_c  # altitude of reference wind speed in meters
        self.direction = direction
        self.reference_wind = reference_wind  # m/s

    def get_wind_at_point(self, position):
        """
        updatess wind vector (vx, vy, vz) at given position (x, y, z)
        Wind speed varies with altitude.
        """
        x, y, z = position

        mag = self.reference_wind * (z / self.z_c)**(0.2)  # power law wind profile

        vx = mag * math.cos(self.direction * (math.pi/180)) # wind magnitude in x-direction (converts deg to radians)
        vy = mag * math.sin(self.direction * (math.pi/180)) # y direction
        
        if vy < 0.001 and vy > -0.001:
            vy = 0.0
        if vx < 0.001 and vx > -0.001:
            vx = 0.0

        return (vx, vy, 0.0)  # assume no vertical wind for now
    
    def get_wind_direction(self):
        vx, vy, vz = self.get_wind_at_point((0, 0, self.z_c))
        mag = (vx**2 + vy**2 + vz**2)**0.5
        if mag == 0:
            return (0, 0, 0)
        #print("Wind Direction: " + str((vx/mag, vy/mag, vz/mag)))
        return (vx/mag, vy/mag, vz/mag)
    
    def get_max_wind_alt(self):
        return self.z_c
    
    def set_baseline_wind(self, new_baseline):
        self.baseline_wind = new_baseline

    def print_map(self):
        # Print wind speed at various altitudes
        print("Altitude (m) | Wind Speed (m/s)")
        for z in range(0, 301, 10):
            vx, vy, vz = self.get_wind_at_point((0, 0, z))
            speed = (vx**2 + vy**2 + vz**2)**0.5
            print(f"{z:12} | {speed:15.2f}")

        # showing plot of wind speed vs altitude
        altitudes = list(range(0, 301, 1))
        speeds = [] 
        for z in altitudes:
            vx, vy, vz = self.get_wind_at_point((0, 0, z))
            speed = (vx**2 + vy**2 + vz**2)**0.5
            speeds.append(speed)
        plt.plot(altitudes, speeds, linewidth=2)
        plt.ylabel('Wind Speed (m/s)')
        plt.xlabel('Altitude (m)')
        plt.title('Wind Speed vs Altitude', fontsize=14, fontweight='bold')
        plt.grid()
        plt.savefig('artifacts/plots/wind_speed_vs_altitude.png')
        #plt.show()
        

"""
# test wind field
def main():
    wind1 = Wind()


main()
"""