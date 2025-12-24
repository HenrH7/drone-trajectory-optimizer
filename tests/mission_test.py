import sys
from pathlib import Path

# Add parent directory to path to allow importing from src
sys.path.insert(0, str(Path(__file__).parent.parent))

import matplotlib.pyplot as plt

from src.optimizer_package.path_optimizer import PathOptimizer
from tests.wind import Wind

def test_mission(points, wind):
    print("-"*60)
    mission_path_points = []
    energy_for_segments = []
    time_for_segments = []
    straight_line_energy_for_segments = []
    
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    dir_wind = wind.get_wind_direction()
    max_wind_alt = wind.get_max_wind_alt()

    # Labels
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m) Altitude")
    ax.set_title("3D Drone Flight Path")

    ax.quiver(
        points[0][0], points[0][1], wind.get_max_wind_alt(),
        dir_wind[0], dir_wind[1], dir_wind[2],
        length=100, color='red', normalize=True, label=('Wind: ' + str(wind.get_wind_at_point((0, 0, max_wind_alt))) + " at: " + str(max_wind_alt) + "m")
    )

    for i in range(len(points)-1):
        start = points[i]
        end = points[i+1]

        d1 = PathOptimizer(
            rho=1.225,  # kg/m^3
            S=0.02,  # m^2
            CD0=1.1,  # drag coefficient
            mass=1.6,  # kg
            battery_capacity_Wh=200,  # Wh
            motor_power_limit_W=1500,  # W
            start_point=start,
            end_point=end,
            max_time=1000, # seconds
            wind_field=wind # pass in wind field
        )

        result, time, E_Wh, path_points = d1.optimize_mission()

        # plot segment
        xs = [p[0] for p in path_points]
        ys = [p[1] for p in path_points]
        zs = [p[2] for p in path_points]

        ax.plot(xs, ys, zs, marker='o', color='blue', label='_Flight Path')

        if i == 0:
            ax.scatter(*start, color='green', s=100, label="Start")
        else:
            ax.scatter(*start, color='black', s=100, label=f"_Waypoint")

        # Straight line from start to end
        ax.plot([xs[0], xs[-1]], [ys[0], ys[-1]], [zs[0], zs[-1]], marker='o', color='teal', markersize=10, label='_Straight Line Path')

        energy_for_segments.append(E_Wh)
        time_for_segments.append(time)
        mission_path_points.extend(path_points[:-1])
        
        # Create straight line control points (start and end only)
        straight_line_x = [
            result.x[0], result.x[1],  # Use same speeds as optimized path
            start[0], start[1], start[2],  # Start point
            end[0], end[1], end[2]  # End point
        ]
        
        straight_E_J, straight_time, straight_path = d1.compute_path(straight_line_x)
        straight_E_Wh = straight_E_J / 3600
        straight_line_energy_for_segments.append(straight_E_Wh)

        # data for each segment
        print(f"\nSegment {i+1}: from {start} to {end}")
        print("Time (s):", time)
        print("Energy (Wh):", E_Wh)
        print(f"Straight Line Energy (Wh): {straight_E_Wh:.2f}")
        print(f"Start speed (m/s): ", result.x[0])  # print speeds
        print(f"End speed (m/s): ", result.x[1])  # print
        print("")

    mission_path_points.append(points[-1])  # add final point
    ax.scatter(*points[-1], color='red', s=100, label="End")

    print("\n--- Mission Summary ---")
    print("Total Mission Time (s):", sum(time_for_segments))
    print("Total Mission Energy (Wh):", sum(energy_for_segments))

    print("\nStraight Line Comparison for Entire Mission:")
    print("Total Straight Line Energy (Wh):", sum(straight_line_energy_for_segments))
    print("Total Energy Saved by Optimization (Wh):", sum(straight_line_energy_for_segments) - sum(energy_for_segments))
    print(f"Percentage Energy Savings: {((sum(straight_line_energy_for_segments) - sum(energy_for_segments)) / sum(straight_line_energy_for_segments) * 100):.2f}%")

    ax.legend()
    plt.tight_layout()
    plt.savefig('artifacts/plots/flight_path.png', dpi=300, bbox_inches='tight', pad_inches=0.5)
    plt.show()
    
# Define mission points with multiple waypoints
mission_points = [
    (-500, 0, 0), # Start point
    (500, 0, 100), 
    #(1000, 400, 150), 
    (500, 800, 200),
    (-500, 800, 0)  # End point
]

def main():
    test_mission(points=mission_points, wind=Wind(200, 0, 20))

main()