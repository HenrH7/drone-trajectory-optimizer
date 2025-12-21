# -*- coding: utf-8 -*-
"""
Drone Mission Optimization

Created on Thursday Oct 2 2025\

Modelling Multi Rotor Drone Path and Energy Consumption

"""

import math
import numpy as np
import bezier
from scipy.optimize import minimize

class PathOptimizer:

    def __init__(self, rho, S, CD0, mass, battery_capacity_Wh, motor_power_limit_W
                 , start_point, end_point, max_time=120, wind_field=None, num_points=10,
                 motor_efficiency=0.85, battery_efficiency=0.9, figure_of_merit=0.7, 
                 prop_diameter=0.2, num_rotors=4, VERBOSE=False):
        # Constants
        self.g = 9.81  # m/s^2
        self.rho = rho # air density (kg/m^3)

        # Aerodynamic parameters
        self.S = S
        self.CD0 = CD0

        # Drone specs
        self.mass = mass
        self.battery_capacity_Wh = battery_capacity_Wh
        self.motor_power_limit_W = motor_power_limit_W

        # Efficiency parameters
        self.motor_efficiency = motor_efficiency  # motor electrical to mechanical efficiency
        self.battery_efficiency = battery_efficiency  # battery discharge efficiency
        self.figure_of_merit = figure_of_merit  # rotor efficiency (0.6-0.75 typical)

        # Rotor parameters
        self.prop_diameter = prop_diameter
        self.num_rotors = num_rotors
        self.area_rotors = self.num_rotors * (math.pi * (self.prop_diameter / 2)**2)  # total rotor disk area

        # Mission profile
        self.wind_field = wind_field  # Wind field object
        self.start_point = start_point
        self.end_point = end_point
        self.max_time = max_time  # max mission time in seconds
        self.num_points = num_points  # number of path segments for integration

        self.max_power_encountered = 0  # track max power during path

        self.VERBOSE = VERBOSE

    def power_required(self, v_x, v_y, v_z, position):
        """
        Compute power required for given velocity
        
        Parameters:
        v_x, v_y, v_z: velocity components in m/s (ground frame)
        
        Returns:
        Power in Watts (electrical power from battery)
        """

        # Airspeed components (ground speed - wind)
        v_wind = self.wind_field.get_wind_at_point(position) # Find wind at current position
        v_air_x = v_x - v_wind[0]
        v_air_y = v_y - v_wind[1]
        v_air_z = v_z - v_wind[2]

        v_h = math.sqrt(v_air_x**2 + v_air_y**2)  # horizontal airspeed

        # Thrust required
        W = self.mass * self.g
        D = 0.5 * self.rho * v_h**2 * self.CD0 * self.S
        T = math.sqrt(W**2 + D**2) # When flying forward, drone tilts to overcome drag

        # Induced power in hover
        v_i = math.sqrt(T / (2 * self.rho * self.area_rotors))  # induced velocity in hover    
        mu = v_h / (v_i + 1e-6) # avoid div by zero (advance ratio)
        P_induced = T * v_i / math.sqrt(1 + mu**2)

        # Pofile power (blade drag)
        P_profile = 0.15 * W * v_i

        # Parasitic power (body drag)
        P_parasitic = D * v_h

        # Climb power (if climbing) aka potential energy rate
        P_climb = W * v_air_z if v_air_z > 0 else 0

        # mechaical power before efficiencies
        P_mechanical = P_induced + P_profile + P_parasitic + P_climb

        # Account for rotor efficiency (figure of merit)
        P_mechanical = P_mechanical / self.figure_of_merit

        # Account for motor efficiency
        P_electrical = P_mechanical / self.motor_efficiency

        # Ensure non-negative
        P_electrical = max(P_electrical, 0)

        return P_electrical  # Watts from battery
    
    def bezier_curve(self, control_points, num_points=30):
        """
        Generate Bezier curve points from control points
        """
        nodes = np.asfortranarray(control_points).T
        curve = bezier.Curve(nodes, degree=len(control_points) - 1)
        s_vals = np.linspace(0, 1, num_points)
        points = curve.evaluate_multi(s_vals).T
        return points
    
    def compute_path(self, x):
        """
        Given optimization variables x, compute total energy and time
        Returns:
        E_total_J: total energy in Joules
        time_total: total time in seconds
        path_points: sampled points along the path
        """

        E_total_J = 0.0
        time_total = 0.0
        self.max_power_encountered = 0.0  # reset for this computation

        # Two speeds: start and end (linear interpolation along path)
        v_start = x[0]
        v_end = x[1]

        # Bezier curve control points are now at indices 2..8
        point1 = (x[2], x[3], x[4])
        point2 = (x[5], x[6], x[7])
        control_points = [self.start_point, point1, point2, self.end_point]
        
        # Sample the points along the Bezier curve
        path_points = self.bezier_curve(control_points, num_points=self.num_points)

        N = len(path_points)
        for i in range(N - 1):
            p0 = path_points[i]
            p1 = path_points[i + 1]

            # Distance between points
            dist = math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2 + (p1[2] - p0[2])**2)
            # local speed interpolation (use segment midpoint index for better mapping)
            s = i / max(N - 2, 1)   # s in [0,1] across segments
            speed_local = v_start + (v_end - v_start) * s
            # guard speed
            speed_local = max(1e-3, speed_local)

            dt = dist / speed_local  # time for this segment

            # velocity components (ground-frame)
            v_x = (p1[0] - p0[0]) / dt
            v_y = (p1[1] - p0[1]) / dt
            v_z = (p1[2] - p0[2]) / dt

            # Get power
            power = self.power_required(v_x, v_y, v_z, p0)

            # Track maximum power for power constraint
            self.max_power_encountered = max(self.max_power_encountered, power)

            # Add to total energy
            E_segment_J = power * dt
            E_total_J += E_segment_J

            # Add to total time
            time_total += dt

        return E_total_J, time_total, path_points
    
    def objective_function(self, x):
        # Objective: minimize energy consumption (return e)
        e, t, _ = self.compute_path(x)
        return e
        
    def constraint_battery(self, x):
        E_total_J, _, _ = self.compute_path(x)
        E_total_Wh = E_total_J / 3600
        safety_factor = 0.80  # Use only 80% of battery
        return (self.battery_capacity_Wh * safety_factor) - E_total_Wh

    def constraint_motor_power(self, x):
        # Limit instantaneous motor power
        _, _, _ = self.compute_path(x)
        return self.motor_power_limit_W - self.max_power_encountered 

    def constraint_time(self, x):
        # Limit total mission time
        _, time, _ = self.compute_path(x)
        return self.max_time - time

    def optimize_mission(self):
        # Initial guess 
        # [speed_start, speed_end, point1_x, point1_y, point1_z, point2_x, point2_y, point2_z]
        x0 = [15.0, 15.0]
        
        # Add control points (2 points in 3D), and interpolate initial guesses
        for point in range(2):  # 2 points
            for i in range(2):  # x, y
                coord = self.start_point[i] + (self.end_point[i] - self.start_point[i]) * (point + 1) / 3
                x0.append(coord) 
            x0.append(max(self.start_point[2], self.end_point[2]) + 20)  # z initial guess

        # Bounds

        z_start = self.start_point[2]
        z_end = self.end_point[2]
        z_min = min(z_start, z_end)
        z_max = max(z_start, z_end)
        
        bounds = [
            (15.0, 30.0),  # speed bound start
            (15.0, 30.0),  # speed bound end

            # control point 1 (allow some deviation, 50m)
            (min(self.start_point[0], self.end_point[0]) - 40, max(self.start_point[0], self.end_point[0]) + 40),  # x1
            (min(self.start_point[1], self.end_point[1]) - 40, max(self.start_point[1], self.end_point[1]) + 40),  # y1
            (max(0, z_min - 40), z_max + 40),  # z1

            # control point 2 (allow some deviation, 50m)
            (min(self.start_point[0], self.end_point[0]) - 40, max(self.start_point[0], self.end_point[0]) + 40),  # x2
            (min(self.start_point[1], self.end_point[1]) - 40, max(self.start_point[1], self.end_point[1]) + 40),  # y2 
            (max(0, z_min - 40), z_max + 40),  # z2
        ]
        
        # Constraints
        constraints = [
            {'type': 'ineq', 'fun': self.constraint_battery},
            {'type': 'ineq', 'fun': self.constraint_motor_power},
            {'type': 'ineq', 'fun': self.constraint_time}
        ]

        options = {'disp': False, 'maxiter': 500}

        # Run optimization
        result = minimize(self.objective_function, x0, constraints=constraints, bounds=bounds, method='SLSQP', options=options)

        if result.success is False:
            print("Optimization failed:", result.message)
        else: 
            e_J, time, path_points = self.compute_path(result.x)
            total_E_Wh = e_J / 3600

            if self.VERBOSE:
                print("\n" + "="*50)
                print("OPTIMIZATION RESULTS")
                print("="*50)
                print(f"Start Speed: {result.x[0]:.2f} m/s")
                print(f"End Speed:   {result.x[1]:.2f} m/s")
                print(f"Total Time:      {time:.2f} s")
                print(f"Total Energy:    {total_E_Wh:.2f} Wh")
                print(f"Battery Used:    {(total_E_Wh/self.battery_capacity_Wh)*100:.1f}%")
                print(f"\nControl Point 1: ({result.x[2]:.1f}, {result.x[3]:.1f}, {result.x[4]:.1f})")
                print(f"Control Point 2: ({result.x[5]:.1f}, {result.x[6]:.1f}, {result.x[7]:.1f})")
                
                # Check constraints
                print(f"\nConstraint Margins:")
                print(f"  Battery:  {self.constraint_battery(result.x):.2f} Wh remaining")
                print(f"  Power:    {self.constraint_motor_power(result.x):.2f} W margin")
                print(f"  Time:     {self.constraint_time(result.x):.2f} s remaining")
                
                # Analyze path shape
                altitudes = [p[2] for p in path_points]
                print(f"\nPath Altitude Profile:")
                print(f"  Max altitude: {max(altitudes):.1f} m")
                print(f"  Min altitude: {min(altitudes):.1f} m")
                print(f"  Start:        {altitudes[0]:.1f} m")
                print(f"  End:          {altitudes[-1]:.1f} m")

            return result, time, total_E_Wh, path_points
        return None, None, None, None
