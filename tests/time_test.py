import sys
from pathlib import Path

# Add parent directory to path to allow importing from src
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import matplotlib.pyplot as plt
import time

from tests.wind import Wind
from src.optimizer_package.path_optimizer import PathOptimizer

def time_test(tests=48): 
    points = [
        (0, 0, 0),      # Start point
        (1000, 0, 200), # End point
    ]

    start_times = []
    end_times = []
    run_times = []
    
    wind_speeds = [0, 5, 10, 15, 20, 25]
    wind_directions = [0, 45, 90, 135, 180, 225, 270, 315]
    
    # Store results for analysis
    results = []

    print(f"\n{'='*70}")
    print(f"RUNNING {tests} OPTIMIZATION TESTS")
    print(f"Testing {len(wind_speeds)} wind speeds × {len(wind_directions)} directions")
    print(f"{'='*70}\n")

    for i in range(tests):
        #cycle through combinations
        wind_idx = i % len(wind_speeds)
        dir_idx = (i // len(wind_speeds)) % len(wind_directions)
        
        wind_speed = wind_speeds[wind_idx]
        wind_dir = wind_directions[dir_idx]
        
        print(f"\n--- Test {i+1}/{tests}: Wind {wind_speed} m/s @ {wind_dir}° ---")
        
        # Create wind field
        wind_field = Wind(
            z_c=10,
            direction=wind_dir,
            reference_wind=wind_speed,
        )
        
        # Run optimization
        start_time = time.time()
        result = run_mission(wind_field, points)
        end_time = time.time()
        
        runtime = end_time - start_time
        
        start_times.append(start_time)
        end_times.append(end_time)
        run_times.append(runtime)
        
        # Store results
        results.append({
            'test_num': i + 1,
            'wind_speed': wind_speed,
            'wind_direction': wind_dir,
            'runtime': runtime,
            'success': result is not None
        })
        
        print(f"Optimization took {runtime:.2f} seconds")
        
        # Progress indicator
        if (i + 1) % 8 == 0:  # After each full direction sweep
            print(f"\n{'='*70}")
            print(f"Completed {i+1}/{tests} tests ({(i+1)/tests*100:.1f}%)")
            print(f"Average time: {np.mean(run_times):.2f} s")
            print(f"{'='*70}")

    # Final statistics
    print(f"\n{'='*70}")
    print(f"ALL TESTS COMPLETE")
    print(f"{'='*70}")
    print(f"Total tests:     {len(run_times)}")
    print(f"Successful:      {sum(1 for r in results if r['success'])}")
    print(f"Failed:          {sum(1 for r in results if not r['success'])}")
    print(f"Total time:      {sum(run_times):.2f} s")
    print(f"Average time:    {np.mean(run_times):.2f} s")
    print(f"Min time:        {min(run_times):.2f} s")
    print(f"Max time:        {max(run_times):.2f} s")
    print(f"{'='*70}\n")
    
    # Plot results
    plot_run_times(run_times, results, wind_speeds, wind_directions)
    
    return results

def run_mission(wind_field, points):
    """Run a single mission optimization"""
    d1 = PathOptimizer(
        rho=1.225,                  # kg/m^3
        S=0.02,                     # m^2
        CD0=1.1,                    # drag coefficient
        mass=1.6,                   # kg
        battery_capacity_Wh=200,    # Wh
        motor_power_limit_W=1500,   # W
        start_point=points[0],
        end_point=points[-1],
        max_time=1000,              # seconds
        wind_field=wind_field,      # pass in wind field
        VERBOSE=False               # Don't print details for batch runs
    )

    result, time_taken, energy, path = d1.optimize_mission()
    
    return result 
    
def plot_run_times(run_times, results, wind_speeds, wind_directions):
    """Create visualization of run times"""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Run times over all tests
    ax1 = axes[0, 0]
    ax1.plot(run_times, marker='o', linestyle='-', linewidth=1, markersize=4)
    ax1.axhline(y=np.mean(run_times), color='r', linestyle='--', 
                label=f'Mean: {np.mean(run_times):.2f}s')
    ax1.set_title('Optimization Run Times', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Test Number')
    ax1.set_ylabel('Time (seconds)')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Run times by wind speed
    ax2 = axes[0, 1]
    wind_speed_times = {ws: [] for ws in wind_speeds}
    for r in results:
        wind_speed_times[r['wind_speed']].append(r['runtime'])
    
    avg_times_by_speed = [np.mean(wind_speed_times[ws]) for ws in wind_speeds]
    ax2.bar(range(len(wind_speeds)), avg_times_by_speed, 
            tick_label=[f"{ws} m/s" for ws in wind_speeds])
    ax2.set_title('Average Run Time by Wind Speed', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Wind Speed')
    ax2.set_ylabel('Average Time (seconds)')
    ax2.grid(True, alpha=0.3, axis='y')
    
    # Run times by direction
    ax3 = axes[1, 0]
    direction_times = {d: [] for d in wind_directions}
    for r in results:
        direction_times[r['wind_direction']].append(r['runtime'])
    
    avg_times_by_dir = [np.mean(direction_times[d]) for d in wind_directions]
    ax3.bar(range(len(wind_directions)), avg_times_by_dir,
            tick_label=[f"{d}°" for d in wind_directions])
    ax3.set_title('Average Run Time by Wind Direction', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Wind Direction')
    ax3.set_ylabel('Average Time (seconds)')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Histogram of run times
    ax4 = axes[1, 1]
    ax4.hist(run_times, bins=20, edgecolor='black', alpha=0.7)
    ax4.axvline(x=np.mean(run_times), color='r', linestyle='--', linewidth=2,
                label=f'Mean: {np.mean(run_times):.2f}s')
    ax4.axvline(x=np.median(run_times), color='g', linestyle='--', linewidth=2,
                label=f'Median: {np.median(run_times):.2f}s')
    ax4.set_title('Distribution of Run Times', fontsize=12, fontweight='bold')
    ax4.set_xlabel('Time (seconds)')
    ax4.set_ylabel('Frequency')
    ax4.legend()
    ax4.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('artifacts/plots/optimization_performance.png', dpi=300, bbox_inches='tight')
    print("Performance plot saved to: optimization_performance.png")
    plt.show()


def main():
    # Test all combinations: 6 speeds × 8 directions = 48 tests
    results = time_test(tests=48)
    

main()