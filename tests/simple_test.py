import sys
from pathlib import Path

# Add parent directory to path to allow importing from src
sys.path.insert(0, str(Path(__file__).parent.parent))

import matplotlib.pyplot as plt
import math

from src.optimizer_package.path_optimizer import PathOptimizer
from tests.wind import Wind

# simple path points for testing
points = [
    (0, 0, 0),  # Start point
    (1000, 0, 200)  # End point
]


def get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)

def plot_flight_path(path_points, wind_field_obj=None):
    # Plotting the 3D flight path
    all_points = path_points
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    zs = [p[2] for p in all_points]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(xs, ys, zs, marker='o', color='blue', label='Flight Path')

    if wind_field_obj is not None:
        wind_field = wind_field_obj
    else:
        wind_field = Wind()

    dir_wind = wind_field.get_wind_direction()
    max_wind_alt = wind_field.get_max_wind_alt()

    ax.quiver(
        all_points[0][0], all_points[0][1], wind_field.get_max_wind_alt(),
        dir_wind[0], dir_wind[1], dir_wind[2],
        length=100, color='red', normalize=True, label=('Wind: ' + str(wind_field.get_wind_at_point((0, 0, max_wind_alt))) + " at: " + str(max_wind_alt) + "m")
    )

    # Straight line from start to end
    ax.plot([xs[0], xs[-1]], [ys[0], ys[-1]], [zs[0], zs[-1]], marker='o', color='green', markersize=10, label='Straight Line Path')

    # Highlight start and end
    ax.scatter(*all_points[0], color='green', s=100, label='Start')
    ax.scatter(*all_points[-1], color='red', s=100, label='End')

    # Labels
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m) Altitude")
    ax.set_title("3D Drone Flight Path")
    ax.legend()

    plt.savefig('artifacts/plots/flight_path.png', dpi=300, bbox_inches='tight')
    plt.show()

def simple_test(wind_field=None, plot=False):

    wind_field = wind_field if wind_field is not None else Wind()

    d1 = PathOptimizer(
        rho=1.225,  # kg/m^3
        S=0.02,  # m^2
        CD0=1.1,  # drag coefficient
        mass=1.6,  # kg
        battery_capacity_Wh=200,  # Wh
        motor_power_limit_W=1500,  # W
        start_point=points[0],
        end_point=points[-1],
        max_time=1000, # seconds
        wind_field=wind_field # pass in wind field
    )

    result, total_time, total_E_Wh, path_points = d1.optimize_mission()

    print("\n--- Optimization Results ---")
    print("Total Time (s):", total_time)
    print("Total Energy (Wh):", total_E_Wh)
    print("start speed (m/s): ", result.x[0])  # print speeds
    print("end speed (m/s): ", result.x[1])  # print speeds

    # Calculate straight-line energy consumption
    print("\n--- Straight Line Comparison ---")
    
    # Create straight line control points (start and end only)
    straight_line_x = [
        result.x[0], result.x[1],  # Use same speeds as optimized path
        points[0][0], points[0][1], points[0][2],  # Start point
        points[-1][0], points[-1][1], points[-1][2]  # End point (acts as both control points)
    ]
    
    straight_E_J, straight_time, straight_path = d1.compute_path(straight_line_x)
    straight_E_Wh = straight_E_J / 3600
    
    print(f"Straight Line Energy (Wh): {straight_E_Wh:.2f}")
    print(f"Straight Line Time (s): {straight_time:.2f}")
    print(f"Energy Saved by Optimization (Wh): {straight_E_Wh - total_E_Wh:.2f}")
    print(f"Percentage Energy Savings: {((straight_E_Wh - total_E_Wh) / straight_E_Wh * 100):.2f}%")

    # Plot flight path
    if plot:
        plot_flight_path(path_points, wind_field_obj=wind_field)

    return straight_E_Wh, straight_time, total_E_Wh, total_time, path_points

def main(): 
    wind_field_1 = Wind(200, 0, 20)
    simple_test(wind_field=wind_field_1, plot=True)
    wind_field_1.print_map()
    
main()
