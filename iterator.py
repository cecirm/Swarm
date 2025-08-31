#!/usr/bin/env python3
"""
Batch simulation tester for swarm formation analysis.

Runs multiple simulations with different seeds to analyze success rates,
V-formation quality, and mission completion efficiency.
"""

import subprocess
import sys
import time
import json
from collections import defaultdict
import argparse

def run_single_simulation(seed):
    """
    Execute a single simulation with validation mode and return detailed results.
    
    Args:
        seed: Random seed for the simulation
        
    Returns:
        dict: Detailed results including success status, timing, and formation quality
    """
    try:
        # Run simulation subprocess with no-visual flag
        result = subprocess.run(
            [sys.executable, 'simulation.py', '--seed', str(seed), '--no-visual'],
            capture_output=True,
            text=True,
            timeout=120  # 120 second timeout per simulation
        )
        
        # Parse output for detailed metrics
        output_lines = result.stdout.strip().split('\n')
        
        # Extract basic success/failure
        success = result.returncode == 0
        
        # Initialize result dictionary
        sim_result = {
            'seed': seed,
            'success': success,
            'output': result.stdout,
            'error': result.stderr,
            'v_formation_frame': None,
            'v_lock_frame': None,
            'threat_activation_frame': None,
            'neutralization_frame': None,
            'settlement_frame': None,
            'total_frames': None,
            'v_formation_quality': 0,
            'mission_efficiency': 0
        }
        
        # Parse detailed metrics from output if available
        for line in output_lines:
            if 'V-formation achieved at frame' in line:
                sim_result['v_formation_frame'] = int(line.split()[-1])
            elif 'V-formation locked at frame' in line:
                sim_result['v_lock_frame'] = int(line.split()[-1])
            elif 'Threat activated at frame' in line:
                sim_result['threat_activation_frame'] = int(line.split()[-1])
            elif 'Neutralization completed at frame' in line:
                sim_result['neutralization_frame'] = int(line.split()[-1])
            elif 'Settlement achieved at frame' in line:
                sim_result['settlement_frame'] = int(line.split()[-1])
            elif 'Total simulation frames' in line:
                sim_result['total_frames'] = int(line.split()[-1])
        
        # Calculate derived metrics
        if success and sim_result['v_formation_frame'] and sim_result['settlement_frame']:
            # V-formation quality: faster formation = higher quality
            v_form_time = sim_result['v_formation_frame']
            sim_result['v_formation_quality'] = max(0, 1000 - v_form_time) / 1000.0
            
            # Mission efficiency: total time to completion
            total_time = sim_result['settlement_frame']
            sim_result['mission_efficiency'] = max(0, 1000 - total_time) / 1000.0
        
        return sim_result
        
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT: Seed {seed} exceeded 120 seconds")
        return {
            'seed': seed,
            'success': False,
            'timeout': True,
            'v_formation_quality': 0,
            'mission_efficiency': 0
        }
    except Exception as e:
        print(f"  ERROR: Seed {seed} failed with exception: {e}")
        return {
            'seed': seed,
            'success': False,
            'error': str(e),
            'v_formation_quality': 0,
            'mission_efficiency': 0
        }

def analyze_detailed_results(results):
    """
    Analyze detailed simulation results for V-formation and efficiency patterns.
    
    Args:
        results: List of detailed result dictionaries
        
    Returns:
        dict: Comprehensive analysis with rankings and recommendations
    """
    successful_results = [r for r in results if r['success']]
    failed_results = [r for r in results if not r['success']]
    
    total_runs = len(results)
    successful_runs = len(successful_results)
    success_rate = (successful_runs / total_runs) * 100 if total_runs > 0 else 0
    
    # Rank successful runs by different criteria
    best_v_formation = sorted(successful_results, 
                             key=lambda x: x.get('v_formation_quality', 0), 
                             reverse=True)[:10]
    
    fastest_missions = sorted(successful_results,
                             key=lambda x: x.get('mission_efficiency', 0),
                             reverse=True)[:10]
    
    # Calculate average metrics for successful runs
    avg_v_formation_time = 0
    avg_mission_time = 0
    if successful_results:
        v_times = [r.get('v_formation_frame', 0) for r in successful_results if r.get('v_formation_frame')]
        mission_times = [r.get('settlement_frame', 0) for r in successful_results if r.get('settlement_frame')]
        
        if v_times:
            avg_v_formation_time = sum(v_times) / len(v_times)
        if mission_times:
            avg_mission_time = sum(mission_times) / len(mission_times)
    
    analysis = {
        'total_runs': total_runs,
        'successful_runs': successful_runs,
        'failed_runs': len(failed_results),
        'success_rate': success_rate,
        'successful_seeds': [r['seed'] for r in successful_results],
        'failed_seeds': [r['seed'] for r in failed_results],
        'best_v_formation_seeds': [r['seed'] for r in best_v_formation],
        'fastest_mission_seeds': [r['seed'] for r in fastest_missions],
        'avg_v_formation_time': avg_v_formation_time,
        'avg_mission_completion_time': avg_mission_time,
        'best_v_formation_details': best_v_formation,
        'fastest_mission_details': fastest_missions
    }
    
    return analysis

def print_enhanced_results(analysis):
    """
    Print comprehensive analysis with V-formation and efficiency rankings.
    
    Args:
        analysis: Enhanced analysis dictionary
    """
    print("\n" + "="*80)
    print("ENHANCED SWARM SIMULATION BATCH ANALYSIS")
    print("="*80)
    
    # Overall statistics
    print(f"\nOVERALL PERFORMANCE:")
    print(f"  Total simulations run: {analysis['total_runs']}")
    print(f"  Successful missions: {analysis['successful_runs']}")
    print(f"  Failed missions: {analysis['failed_runs']}")
    print(f"  Success rate: {analysis['success_rate']:.1f}%")
    
    if analysis['success_rate'] >= 80:
        print(f"  Status: EXCELLENT - High reliability swarm behavior")
    elif analysis['success_rate'] >= 60:
        print(f"  Status: GOOD - Generally reliable with some edge cases")
    elif analysis['success_rate'] >= 40:
        print(f"  Status: MODERATE - Inconsistent performance, needs tuning")
    else:
        print(f"  Status: POOR - Low reliability, major parameter adjustment needed")
    
    # Performance metrics
    if analysis['successful_runs'] > 0:
        print(f"\nPERFORMANCE METRICS:")
        print(f"  Average V-formation time: {analysis['avg_v_formation_time']:.1f} frames")
        print(f"  Average mission completion: {analysis['avg_mission_completion_time']:.1f} frames")
    
    # Top performers for V-formation quality
    if analysis['best_v_formation_details']:
        print(f"\nTOP 10 SEEDS - BEST V-FORMATION QUALITY:")
        print(f"  {'Seed':<6} {'Quality':<8} {'V-Form Frame':<12} {'Total Frames':<12}")
        print(f"  {'-'*6} {'-'*8} {'-'*12} {'-'*12}")
        for result in analysis['best_v_formation_details']:
            quality = result.get('v_formation_quality', 0) * 100
            v_frame = result.get('v_formation_frame', 'N/A')
            total_frame = result.get('settlement_frame', 'N/A')
            print(f"  {result['seed']:<6} {quality:<8.1f} {v_frame:<12} {total_frame:<12}")
    
    # Top performers for mission efficiency
    if analysis['fastest_mission_details']:
        print(f"\nTOP 10 SEEDS - FASTEST MISSION COMPLETION:")
        print(f"  {'Seed':<6} {'Efficiency':<10} {'Settlement Frame':<15} {'V-Lock Frame':<12}")
        print(f"  {'-'*6} {'-'*10} {'-'*15} {'-'*12}")
        for result in analysis['fastest_mission_details']:
            efficiency = result.get('mission_efficiency', 0) * 100
            settlement = result.get('settlement_frame', 'N/A')
            v_lock = result.get('v_lock_frame', 'N/A')
            print(f"  {result['seed']:<6} {efficiency:<10.1f} {settlement:<15} {v_lock:<12}")
    
    # Recommendations
    print(f"\nRECOMMENDATIONS:")
    if analysis['success_rate'] < 50:
        print(f"  - Consider adjusting V_FORMATION or RING parameters")
        print(f"  - Check THREAT_DELAY and convergence timing")
        print(f"  - Verify SENSING_RADIUS and agent interaction ranges")
    else:
        print(f"  - Current parameters show good performance")
    
    print(f"\nOPTIMAL SEEDS FOR DIFFERENT PURPOSES:")
    if analysis['best_v_formation_seeds']:
        print(f"  - Best V-formation demos: {analysis['best_v_formation_seeds'][:5]}")
    if analysis['fastest_mission_seeds']:
        print(f"  - Fastest mission completion: {analysis['fastest_mission_seeds'][:5]}")
    if analysis['failed_seeds']:
        print(f"  - Debug problematic cases: {analysis['failed_seeds'][:5]}")

def run_enhanced_batch_analysis(start_seed=1, end_seed=1000, progress_interval=50):
    """
    Execute enhanced batch simulation testing with detailed metrics collection.
    
    Args:
        start_seed: First seed to test (inclusive)
        end_seed: Last seed to test (inclusive) 
        progress_interval: How often to print progress updates
        
    Returns:
        list: Complete detailed results for all simulations
    """
    print(f"Starting enhanced batch analysis: seeds {start_seed} to {end_seed}")
    print(f"Total simulations to run: {end_seed - start_seed + 1}")
    print("-" * 60)
    
    results = []
    start_time = time.time()
    
    for seed in range(start_seed, end_seed + 1):
        # Run simulation and record detailed result
        result = run_single_simulation(seed)
        results.append(result)
        
        # Progress reporting
        if seed % progress_interval == 0 or seed == end_seed:
            elapsed = time.time() - start_time
            completed = seed - start_seed + 1
            total = end_seed - start_seed + 1
            progress = (completed / total) * 100
            
            current_successes = sum(1 for r in results if r['success'])
            current_rate = (current_successes / completed) * 100
            
            success_indicator = '✓' if result['success'] else '✗'
            v_quality = result.get('v_formation_quality', 0) * 100
            
            print(f"Progress: {completed:4d}/{total} ({progress:5.1f}%) | "
                  f"Success rate: {current_rate:5.1f}% | "
                  f"Elapsed: {elapsed:6.1f}s | "
                  f"Seed {seed}: {success_indicator} (V-Quality: {v_quality:4.1f}%)")
    
    return results

def main():
    """
    Main program for enhanced batch simulation testing with performance analysis.
    """
    parser = argparse.ArgumentParser(description='Enhanced Batch Swarm Simulation Tester')
    parser.add_argument('--start', type=int, default=1, 
                       help='Starting seed value (default: 1)')
    parser.add_argument('--end', type=int, default=50,
                       help='Ending seed value (default: 50)')
    parser.add_argument('--progress', type=int, default=50,
                       help='Progress report interval (default: 50)')
    parser.add_argument('--quick', action='store_true',
                       help='Quick test mode: seeds 1-100 only')
    
    args = parser.parse_args()
    
    # Quick test mode override
    if args.quick:
        start_seed, end_seed = 1, 100
        print("QUICK TEST MODE: Testing seeds 1-100")
    else:
        start_seed, end_seed = args.start, args.end
    
    try:
        # Execute enhanced batch testing
        detailed_results = run_enhanced_batch_analysis(start_seed, end_seed, args.progress)
        
        # Analyze and display enhanced results
        analysis = analyze_detailed_results(detailed_results)
        print_enhanced_results(analysis)
        
        # Save detailed results to JSON file
        output_filename = f"detailed_results_{start_seed}_{end_seed}.json"
        with open(output_filename, 'w') as f:
            json.dump({
                'analysis_summary': analysis,
                'detailed_results': detailed_results
            }, f, indent=2)
        
        print(f"\nDetailed results saved to: {output_filename}")
        
        # Exit with overall success indicator
        if analysis['success_rate'] >= 70:
            print("ENHANCED BATCH TEST PASSED: High success rate achieved")
            sys.exit(0)
        else:
            print("ENHANCED BATCH TEST NEEDS ATTENTION: Low success rate")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\nBatch testing interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nBatch testing failed with error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()