#!/usr/bin/env python3
"""
Batch runner for SUMO experiments.
Supports parallel execution across multiple penetration rates.
"""

import subprocess
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import time
import os
from datetime import datetime
from pathlib import Path

def run_experiment(scenario, controller, penetration, seed=42, log_dir="logs", timeout=None, exp_tag=""):
    """
    Run a single simulation experiment.
    
    Parameters:
    -----------
    scenario : str
        Scenario name (onramp, i24)
    controller : str
        Controller type
    penetration : float
        CAV penetration rate (0.0-1.0)
    seed : int
        Random seed
    log_dir : str
        Directory to save log files
    timeout : int or None
        Timeout in seconds; None means no limit
    exp_tag : str
        Experiment tag (e.g. "mainlane_lc_on") appended to output filenames
    
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
    
    # Python command
    python_cmd = (
        f"python main_controller.py "
        f"--scenario {scenario} "
        f"--cav-controller {controller} "
        f"--penetration {penetration} "
        f"--seed {seed}"
    )
    
    # Append experiment tag if provided
    if exp_tag:
        python_cmd += f" --exp-tag {exp_tag}"
    
    # Full bash command
    full_cmd = f"{env_setup}\n{python_cmd}"
    
    exp_name = f"{scenario}_{controller}_p{penetration}"
    if exp_tag:
        exp_name = f"{exp_name}_{exp_tag}"
    log_file = Path(log_dir) / f"{exp_name}.log"
    
    try:
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 🚀 Starting: {exp_name}")
        print(f"  Log: {log_file}")
        
        # Execute via bash
        with open(log_file, 'w', buffering=1) as log:  # line-buffered
            log.write(f"Experiment: {exp_name}\n")
            log.write(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            log.write(f"Command: {python_cmd}\n")
            log.write("="*70 + "\n\n")
            log.flush()
            
            # Disable Python output buffering
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            
            result = subprocess.run(
                ['bash', '-c', full_cmd],
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                env=env
            )
        
        elapsed = time.time() - start_time
        
        # Append completion info to log
        with open(log_file, 'a') as log:
            log.write(f"\n" + "="*70 + "\n")
            log.write(f"End time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            log.write(f"Elapsed: {elapsed:.1f}s ({elapsed/60:.1f}min)\n")
            log.write(f"Exit code: {result.returncode}\n")
        
        if result.returncode == 0:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] ✅ Done: {exp_name} ({elapsed/60:.1f}min)")
            return (penetration, True, elapsed, "Success")
        else:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] ❌ Failed: {exp_name} (Exit code: {result.returncode})")
            print(f"  Log file: {log_file}")
            return (penetration, False, elapsed, f"Exit code: {result.returncode}")
            
    except subprocess.TimeoutExpired:
        elapsed = time.time() - start_time
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ⏱️ Timeout: {exp_name} (>{elapsed/3600:.1f}h)")
        
        with open(log_file, 'a') as log:
            log.write(f"\n" + "="*70 + "\n")
            log.write(f"⏱️ Timed out: exceeded {timeout}s ({timeout/3600:.1f}h)\n")
        
        return (penetration, False, elapsed, "Timeout")
        
    except Exception as e:
        elapsed = time.time() - start_time
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ❌ Exception: {exp_name} - {str(e)}")
        
        with open(log_file, 'a') as log:
            log.write(f"\n" + "="*70 + "\n")
            log.write(f"Exception: {str(e)}\n")
        
        return (penetration, False, elapsed, str(e))


def main():
    parser = argparse.ArgumentParser(description='Run batch CAV penetration-rate experiments')
    
    parser.add_argument('--scenario', type=str, required=True,
                        choices=['onramp', 'i24'],
                        help='Scenario name')
    
    parser.add_argument('--controller', type=str, required=True,
                        choices=['pcc', 'acc', 'cacc', 'idm',
                                 'li2018', 'gunter2020', 'sun2024', 'zhang2025',
                                 'wen2022', 'vajedi2016', 'mosharafian2022', 'kim2021'],
                        help='Controller type (includes literature-based configurations)')
    
    parser.add_argument('--penetrations', type=str, default='0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9',
                        help='Comma-separated penetration rates (default: 0.0 to 0.9)')
    
    parser.add_argument('--workers', type=int, default=5,
                        help='Number of parallel workers (default: 5)')
    
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed (default: 42)')
    
    parser.add_argument('--timeout', type=int, default=None,
                        help='Per-experiment timeout in seconds (default: no limit). Suggested: 14400 for PCC, 7200 for ACC')
    
    parser.add_argument('--exp-tag', type=str, default='',
                        help='Experiment tag appended to output filenames (e.g. "mainlane_lc_on")')
    
    parser.add_argument('--log-dir', type=str, default='logs',
                        help='Log directory (default: logs)')
    
    args = parser.parse_args()
    
    # Create log directory
    log_dir = Path(args.log_dir)
    log_dir.mkdir(exist_ok=True)
    
    # Parse penetration rate list
    penetrations = [float(p.strip()) for p in args.penetrations.split(',')]
    
    print("="*70)
    print(f"Batch experiment configuration:")
    print(f"  Scenario: {args.scenario}")
    print(f"  Controller: {args.controller}")
    print(f"  Penetrations: {penetrations}")
    print(f"  Workers: {args.workers}")
    print(f"  Random seed: {args.seed}")
    if args.exp_tag:
        print(f"  Experiment tag: {args.exp_tag}")
    print(f"  Timeout: {'None' if args.timeout is None else f'{args.timeout}s ({args.timeout/3600:.1f}h)'}")
    print(f"  Log dir: {log_dir.absolute()}")
    print(f"  Total experiments: {len(penetrations)}")
    print("="*70)
    
    start_time = time.time()
    results = []
    
    # Run experiments in parallel using ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        # Submit all tasks
        futures = {
            executor.submit(run_experiment, args.scenario, args.controller, p, args.seed, args.log_dir, args.timeout, args.exp_tag): p 
            for p in penetrations
        }
        
        # Collect results
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
    
    # Sort results by penetration rate
    results.sort(key=lambda x: x[0])
    
    total_time = time.time() - start_time
    
    # Print summary
    print("\n" + "="*70)
    print("Experiment summary:")
    print("="*70)
    
    success_count = sum(1 for r in results if r[1])
    
    for pen, success, runtime, msg in results:
        status = "✅" if success else "❌"
        print(f"  {status} p={pen:.1f}: {runtime/60:7.1f}min - {msg}")
    
    print("-"*70)
    print(f"Succeeded: {success_count}/{len(results)}")
    print(f"Total time: {total_time/60:.1f}min ({total_time/3600:.1f}h)")
    print(f"Log directory: {log_dir.absolute()}")
    print("="*70)
    
    # Exit with non-zero code if any experiment failed
    if success_count < len(results):
        exit(1)


if __name__ == "__main__":
    main()
