#!/usr/bin/env python3
"""
Batch runner for dashboard.py to export metrics tables.
Only the exported table files matter; the web UI is not used.
"""

import subprocess
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import time
import os
from datetime import datetime
from pathlib import Path

def run_dashboard(scenario, controller, penetration, exclude_lanes, exp_tag="", log_dir="logs_dashboard"):
    """
    Run a single dashboard export task.
    
    Parameters:
    -----------
    scenario : str
        Scenario name (onramp, i24)
    controller : str
        Controller type
    penetration : float
        CAV penetration rate
    exclude_lanes : list
        List of lane IDs to exclude from analysis
    exp_tag : str
        Experiment tag
    log_dir : str
        Log directory
    
    Returns:
    --------
    tuple : (penetration, success, runtime, message)
    """
    start_time = time.time()
    
    # Bash environment setup
    env_setup = """
source ~/myenv311/bin/activate || { echo "Failed to activate venv"; exit 1; }
export SUMO_HOME=$HOME/cav-sumo-1/sumo
export PATH=$SUMO_HOME/bin:$PATH
"""
    
    # Build dashboard command
    cmd_parts = [
        "python dashboard.py",
        f"--scenario {scenario}",
        "--urban no"
    ]
    
    # Add excluded lanes
    for lane in exclude_lanes:
        cmd_parts.append(f"--exclude-lane {lane}")
    
    cmd_parts.extend([
        f"--cav-controller {controller}",
        f"--p {penetration}"
    ])
    
    # Append experiment tag if provided
    if exp_tag:
        cmd_parts.append(f"--exp-tag {exp_tag}")
    
    python_cmd = " ".join(cmd_parts)
    full_cmd = f"{env_setup}\n{python_cmd}"
    
    exp_name = f"{scenario}_{controller}_p{penetration}"
    if exp_tag:
        exp_name = f"{exp_name}_{exp_tag}"
    exp_name = f"{exp_name}_dashboard"
    log_file = Path(log_dir) / f"{exp_name}.log"
    
    try:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 🚀 Exporting: {scenario}_{controller}_p{penetration}")
        print(f"  Log: {log_file}")
        
        with open(log_file, 'w', buffering=1) as log:
            log.write(f"Dashboard task: {exp_name}\n")
            log.write(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            log.write(f"Command: {python_cmd}\n")
            log.write("="*70 + "\n\n")
            log.flush()
            
            # Set environment variables
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            
            result = subprocess.run(
                ['bash', '-c', full_cmd],
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=10800,  # 3-hour timeout; dashboard processing large files may take time
                env=env
            )
        
        elapsed = time.time() - start_time
        
        # Append completion info to log
        with open(log_file, 'a') as log:
            log.write(f"\n" + "="*70 + "\n")
            log.write(f"End time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            log.write(f"Elapsed: {elapsed:.1f}s\n")
            log.write(f"Exit code: {result.returncode}\n")
        
        if result.returncode == 0:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] ✅ Done: {exp_name} ({elapsed:.1f}s)")
            return (penetration, True, elapsed, "Success")
        else:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] ❌ Failed: {exp_name}")
            return (penetration, False, elapsed, f"Exit code: {result.returncode}")
            
    except subprocess.TimeoutExpired:
        elapsed = time.time() - start_time
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ⏱️ Timeout: {exp_name}")
        return (penetration, False, elapsed, "Timeout")
        
    except Exception as e:
        elapsed = time.time() - start_time
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ❌ Exception: {exp_name} - {str(e)}")
        return (penetration, False, elapsed, str(e))


def main():
    parser = argparse.ArgumentParser(description='Run batch dashboard exports to generate metrics tables')
    
    parser.add_argument('--scenario', type=str, required=True,
                        choices=['onramp', 'i24'],
                        help='Scenario name')
    
    parser.add_argument('--controller', type=str, required=True,
                        choices=['pcc', 'acc', 'cacc', 'idm',
                                 'li2018', 'gunter2020', 'sun2024', 'zhang2025',
                                 'wen2022', 'vajedi2016', 'mosharafian2022', 'kim2021'],
                        help='Controller type (includes literature-based configurations)')
    
    parser.add_argument('--penetrations', type=str, default='0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9',
                        help='Comma-separated list of CAV penetration rates')
    
    parser.add_argument('--workers', type=int, default=3,
                        help='Number of parallel workers (default: 3; dashboard is memory-intensive)')
    
    parser.add_argument('--exclude-lanes', type=str, default='',
                        help='Comma-separated lanes to exclude. If omitted, scenario defaults are used.')
    
    parser.add_argument('--exp-tag', type=str, default='',
                        help='Experiment tag (e.g. "mainlane_lc_off", "merging_lc_off")')
    
    parser.add_argument('--log-dir', type=str, default='logs_dashboard',
                        help='Log directory')
    
    args = parser.parse_args()
    
    # Create log directory
    log_dir = Path(args.log_dir)
    log_dir.mkdir(exist_ok=True)
    
    # Parse penetration rates
    penetrations = [float(p.strip()) for p in args.penetrations.split(',')]
    
    # Parse excluded lanes
    if args.exclude_lanes:
        exclude_lanes = [l.strip() for l in args.exclude_lanes.split(',')]
    else:
        # Default excluded lanes per scenario
        if args.scenario == 'i24':
            exclude_lanes = ['E2_0', 'E5_0', 'E3_0', 'E4_0', 'E6_0', 'E1_0', 'E7_0']
        elif args.scenario == 'onramp':
            exclude_lanes = ['ramp_0', 'E2_0']
        else:
            exclude_lanes = []
    
    print("="*70)
    print(f"Batch dashboard configuration:")
    print(f"  Scenario: {args.scenario}")
    print(f"  Controller: {args.controller}")
    print(f"  CAV penetration rate: {penetrations}")
    if args.exp_tag:
        print(f"  Experiment tag: {args.exp_tag}")
    print(f"  Excluded lanes: {exclude_lanes}")
    print(f"  Workers: {args.workers}")
    print(f"  Log directory: {log_dir.absolute()}")
    print(f"  Total tasks: {len(penetrations)}")
    print("="*70)
    print("Note: web port conflicts are ignored; only the exported table files matter.")
    print("="*70)
    
    start_time = time.time()
    results = []
    
    # Run tasks in parallel
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(run_dashboard, args.scenario, args.controller, p, exclude_lanes, args.exp_tag, args.log_dir): p 
            for p in penetrations
        }
        
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
    
    # Sort results by penetration rate
    results.sort(key=lambda x: x[0])
    
    total_time = time.time() - start_time
    
    # Print summary
    print("\n" + "="*70)
    print("Dashboard task summary:")
    print("="*70)
    
    success_count = sum(1 for r in results if r[1])
    
    for pen, success, runtime, msg in results:
        status = "✅" if success else "❌"
        print(f"  {status} p={pen:.1f}: {runtime:6.1f}s - {msg}")
    
    print("-"*70)
    print(f"Succeeded: {success_count}/{len(results)}")
    print(f"Total time: {total_time:.1f}s ({total_time/60:.1f}min)")
    print(f"Log directory: {log_dir.absolute()}")
    
    # Check generated metrics files
    metrics_pattern = f"docs/metrics_{args.controller}_*.json"
    print("-"*70)
    print("Generated metrics files should be in: docs/")
    print(f"  Check: ls docs/metrics_*_{args.controller}_*.json")
    print("="*70)
    
    if success_count < len(results):
        exit(1)


if __name__ == "__main__":
    main()
