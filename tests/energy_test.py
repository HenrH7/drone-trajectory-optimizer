import sys
from pathlib import Path

# Add parent directory to path to allow importing from src
sys.path.insert(0, str(Path(__file__).parent.parent))

import matplotlib.pyplot as plt
import csv

from src.optimizer_package.path_optimizer import PathOptimizer
from tests.wind import Wind


def multiple_path_plot(paths, wind_speeds, title="Multiple Drone Flight Paths"):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for idx, path_points in enumerate(paths):
        xs = [p[0] for p in path_points]
        ys = [p[1] for p in path_points]
        zs = [p[2] for p in path_points]

        ax.plot(xs, ys, zs, marker='o', label=f'Wind {wind_speeds[idx]} m/s')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m) Altitude")
    ax.set_title(title, fontsize=16, fontweight='bold')
    ax.legend()

    plt.savefig(f'artifacts/plots/{title}.png', dpi=300, bbox_inches='tight')
    plt.show()

def compare_tailwind_tests(z_c, wind_range):
    """Compare PathOptimizer vs StraightLineMission across different windspeeds"""
    
    windspeeds = wind_range
    # path points storage
    paths = []
    
    # Storage for results
    optimizer_energy = []
    optimizer_time = []
    straight_energy = []
    straight_time = []
    
    print("Running tests...")
    print("="*60)
    
    # Loop over different tailwind speeds
    for windspeed in windspeeds:
        print(f"\nTesting windspeed: {windspeed} m/s")

        # Set wind baseline
        wind_field = Wind(z_c, 0, windspeed)  # tailwind
        
        straight_energy_x, straight_time_x, optimizer_energy_x, optimizer_time_x, path_points = simple_test(wind_field=wind_field)

        straight_energy.append(straight_energy_x)
        straight_time.append(straight_time_x)
        optimizer_energy.append(optimizer_energy_x)
        optimizer_time.append(optimizer_time_x)

        paths.append(path_points)

        print("-"*40)
    
    # Save to CSV
    with open("artifacts/reports/comparison_results.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Windspeed", "optimizer_energy", "optimizer_time", 
                        "straight_energy", "straight_time"])
        for i, speed in enumerate(windspeeds):
            writer.writerow([speed, optimizer_energy[i], optimizer_time[i],
                           straight_energy[i], straight_time[i]])
    
    print("\n" + "="*60)
    print("Results saved to 'comparison_results.csv'")
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Energy comparison
    ax1.plot(windspeeds, optimizer_energy, 'bo-', linewidth=2, markersize=8, 
             label='Optimized Mission')
    ax1.plot(windspeeds, straight_energy, 'rs--', linewidth=2, markersize=8, 
             label='Straight Line Mission')
    ax1.set_xlabel('Tailwind speed (m/s)', fontsize=12)
    ax1.set_ylabel('Energy (Wh)', fontsize=12)
    ax1.set_title('Energy Consumption vs Tailwind Speed', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # Time comparison
    ax2.plot(windspeeds, optimizer_time, 'bo-', linewidth=2, markersize=8, 
             label='Optimized Mission')
    ax2.plot(windspeeds, straight_time, 'rs--', linewidth=2, markersize=8, 
             label='Straight Line Mission')
    ax2.set_xlabel('Tailwind Speed (m/s)', fontsize=12)
    ax2.set_ylabel('Time (s)', fontsize=12)
    ax2.set_title('Mission Time vs Tailwind Speed', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('artifacts/plots/tailwind_comparison.png', dpi=150, bbox_inches='tight')
    print("Plots saved to 'tailwind_comparison.png'")
    plt.show()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    saved_energy_pcts = []

    for i, speed in enumerate(windspeeds):
        if optimizer_energy[i] is not None:
            energy_diff = abs(optimizer_energy[i] - straight_energy[i])
            energy_pct = 100 * energy_diff / straight_energy[i]
            saved_energy_pcts.append(energy_pct)
            print(f"Speed {speed} m/s: Energy difference = {energy_diff:.2f} Wh ({energy_pct:.1f}%)")

    multiple_path_plot(paths, windspeeds, title="Flight Paths under Different Tailwind Speeds")

    return saved_energy_pcts

def compare_headwind_tests(z_c, wind_range):
    """Compare PathOptimizer vs StraightLineMission across different windspeeds"""
    
    windspeeds = wind_range

    #path points storage
    paths = []
    
    # Storage for results
    optimizer_energy = []
    optimizer_time = []
    straight_energy = []
    straight_time = []
    
    print("Running tests...")
    print("="*60)
    
    for windspeed in windspeeds:
        print(f"\nTesting windspeed: {windspeed} m/s")

        # Set wind baseline
        wind_field = Wind(z_c, 180, windspeed)  # tailwind
        
        straight_energy_x, straight_time_x, optimizer_energy_x, optimizer_time_x, path_points = simple_test(wind_field=wind_field)

        straight_energy.append(straight_energy_x)
        straight_time.append(straight_time_x)
        optimizer_energy.append(optimizer_energy_x)
        optimizer_time.append(optimizer_time_x)

        paths.append(path_points)

        print("-"*40)
    
    # Save to CSV
    with open("artifacts/reports/comparison_results.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Windspeed", "optimizer_energy", "optimizer_time", 
                        "straight_energy", "straight_time"])
        for i, speed in enumerate(windspeeds):
            writer.writerow([speed, optimizer_energy[i], optimizer_time[i],
                           straight_energy[i], straight_time[i]])
    
    print("\n" + "="*60)
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Energy comparison
    ax1.plot(windspeeds, optimizer_energy, 'bo-', linewidth=2, markersize=8, 
             label='Optimized Mission')
    ax1.plot(windspeeds, straight_energy, 'rs--', linewidth=2, markersize=8, 
             label='Straight Line Mission')
    ax1.set_xlabel('Headwind speed (m/s)', fontsize=12)
    ax1.set_ylabel('Energy (Wh)', fontsize=12)
    ax1.set_title('Energy Consumption vs Headwind Speed', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # Time comparison
    ax2.plot(windspeeds, optimizer_time, 'bo-', linewidth=2, markersize=8, 
             label='Optimized Mission')
    ax2.plot(windspeeds, straight_time, 'rs--', linewidth=2, markersize=8, 
             label='Straight Line Mission')
    ax2.set_xlabel('Headwind speed (m/s)', fontsize=12)
    ax2.set_ylabel('Time (s)', fontsize=12)
    ax2.set_title('Mission Time vs Headwind Speed', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('artifacts/plots/headwind_comparison.png', dpi=150, bbox_inches='tight')
    print("Plots saved to 'headwind_comparison.png'")
    plt.show()
    
    # Print summary statistics
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    saved_energy_pcts = []

    for i, speed in enumerate(windspeeds):
        if optimizer_energy[i] is not None:
            energy_diff = abs(optimizer_energy[i] - straight_energy[i])
            energy_pct = 100 * energy_diff / straight_energy[i]
            saved_energy_pcts.append(energy_pct)
            print(f"Speed {speed} m/s: Energy difference = {energy_diff:.2f} Wh ({energy_pct:.1f}%)")

    multiple_path_plot(paths, windspeeds, title="Flight Paths under Different Headwind Speeds")

    return saved_energy_pcts  

def simple_test(wind_field=None):

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

    return straight_E_Wh, straight_time, total_E_Wh, total_time, path_points

def test_energy():
    # comparing energy and time at different wind speeds in both headwind and tailwind
    z_c = 200
    wind_range = range(0, 32, 5)
    savings_head = compare_headwind_tests(z_c, wind_range)
    savings_tail = compare_tailwind_tests(z_c, wind_range)
    

    print("\n" + "="*60)
    print("AVERAGE ENERGY SAVINGS")
    print(sum(savings_head)/len(savings_head), "% for headwind tests")
    print(sum(savings_tail)/len(savings_tail), "% for tailwind tests")

# path points for testing
points = [
    (0, 0, 0),  # Start point
    (1000, 0, 200)  # End point
]

def main():
    test_energy()

main()
