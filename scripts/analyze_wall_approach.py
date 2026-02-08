#!/usr/bin/env python3
"""Analyze wall approach test data. Shows summary + braking phase detail."""

import pickle
import sys
import os
import glob

def analyze(directory, tail_count=25):
    pkl_path = os.path.join(directory, 'wall_approach_data.pkl')
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    
    samples = data['data']
    target = data['target_distance']
    
    print(f"=== {directory} ===")
    print(f"Start: {data.get('start_distance', float('nan')):.3f}m  "
          f"Final: {data.get('final_distance', float('nan')):.3f}m  "
          f"Target: {target:.3f}m")
    print(f"Error: {data.get('distance_error', float('nan'))*1000:.1f}mm  "
          f"Peak v: {data.get('peak_velocity', float('nan')):.3f}m/s")
    print(f"Params: speed={data.get('max_speed')}, "
          f"accel={data.get('accel')}, decel={data.get('decel')}")
    print()
    
    # Filter to valid samples (non-NaN v_target)
    valid = [d for d in samples if d['v_target_linear'] == d['v_target_linear']]
    
    if not valid:
        print("No valid samples!")
        return
    
    # Show last N valid samples (braking phase)
    tail = valid[-tail_count:]
    
    header = (f"{'time':>6} {'tof_d':>7} {'d2tgt':>6} {'v_act':>7} "
              f"{'v_tgt':>6} {'a_tgt':>6} {'Lmode':>5} {'Rmode':>5} "
              f"{'Lcmd':>7} {'Rcmd':>7}")
    print(header)
    print('-' * len(header))
    
    for d in tail:
        d2t = d['tof_distance'] - target
        print(f"{d['time']:6.3f} {d['tof_distance']:7.4f} {d2t:6.3f} "
              f"{d['v_actual']:7.4f} {d['v_target_linear']:6.3f} "
              f"{d['accel_target']:6.2f} {d['left_motor_mode']:5d} "
              f"{d['right_motor_mode']:5d} {d['left_motor_command']:7.3f} "
              f"{d['right_motor_command']:7.3f}")
    print()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Analyze wall approach test data')
    parser.add_argument('dirs', nargs='*', help='Test directories to analyze (default: latest)')
    parser.add_argument('-n', '--count', type=int, default=1,
                        help='Number of latest runs to show (default: 1)')
    parser.add_argument('-t', '--tail', type=int, default=25,
                        help='Number of trailing samples to show (default: 25)')
    parser.add_argument('-a', '--all', action='store_true',
                        help='Show all samples, not just tail')
    args = parser.parse_args()
    
    if args.dirs:
        directories = args.dirs
    else:
        # Find latest N wall_approach directories
        pattern = os.path.join('test_outputs', 'wall_approach_*')
        directories = sorted(glob.glob(pattern), reverse=True)[:args.count]
        directories.reverse()  # chronological order
    
    if not directories:
        print("No wall approach test data found")
        sys.exit(1)
    
    tail = 999999 if args.all else args.tail
    for d in directories:
        analyze(d, tail_count=tail)


if __name__ == '__main__':
    main()
