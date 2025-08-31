# Swarm Formation Simulation

A comprehensive multi-agent simulation system that demonstrates progressive swarm intelligence behaviors, from individual exploration to coordinated V-formation flight and threat neutralization.

## Overview

This simulation models a swarm of autonomous agents that exhibit four distinct behavioral phases:

1. **SEARCH** - Individual random exploration and discovery
2. **GROUP** - Collective movement and flocking behavior  
3. **LINE** - V-formation flight with velocity synchronization
4. **RING** - Coordinated threat encirclement and neutralization

The system implements advanced swarm algorithms including leader selection, formation locking, and distributed coordination without centralized control.

## Project Structure

```
swarm-simulation/
├── simulation.py          # Main simulation engine with CLI interface
├── batch_tester.py        # Automated testing and analysis tool
├── visualization.py       # Real-time animation and visualization
├── settings.py           # Configuration parameters
├── agents/
│   └── agent.py          # Individual agent behavior and decision making
└── env/
    └── environment.py    # World physics and threat management
```

## Quick Start

### Basic Usage

```bash
# Run single simulation with visualization
python simulation.py

# Run with specific seed for reproducible results
python simulation.py --seed 42

# Run validation mode (no visualization, reports success/failure)
python simulation.py --no-visual

# Combine seed and validation mode
python simulation.py --seed 123 --no-visual
```

### Batch Testing and Analysis

```bash
# Test 100 different seeds and analyze performance
python batch_tester.py --quick

# Full analysis: test seeds 1-1000
python batch_tester.py

# Custom range testing
python batch_tester.py --start 50 --end 200

# Monitor progress every 25 seeds
python batch_tester.py --progress 25
```

## Command Line Interface

### `simulation.py` Options

| Option | Description | Example |
|--------|-------------|---------|
| `--seed N` | Set random seed for reproducible results | `--seed 42` |
| `--no-visual` | Run validation mode without visualization | `--no-visual` |

**Exit Codes:**
- `0` - Mission completed successfully
- `1` - Mission failed (threat not neutralized)

### `batch_tester.py` Options

| Option | Description | Default |
|--------|-------------|---------|
| `--start N` | Starting seed value | 1 |
| `--end N` | Ending seed value | 1000 |
| `--progress N` | Progress report interval | 50 |
| `--quick` | Quick test mode (seeds 1-100) | disabled |

## Configuration

### Key Settings (`settings.py`)

#### World Parameters
```python
WORLD_WIDTH = 100.0         # World dimensions (meters)
WORLD_HEIGHT = 100.0        
NUM_AGENTS = 10             # Swarm size
MAX_SPEED = 2.0            # Agent maximum velocity (m/s)
```

#### Formation Parameters
```python
V_SPREAD_ANGLE = 45°        # V-formation wing angle
V_DISTANCE = 25.0          # Spacing between agents on wings
RING_RADIUS = 10.0         # Threat encirclement radius
THREAT_DELAY = 120.0       # Seconds to wait before threat activation
```

#### Sensing and Detection
```python
SENSING_RADIUS = 10.0       # Local neighbor detection range
SIGHT_RADIUS = 50.0        # Extended visibility for leadership
THREAT_DETECTION_RADIUS = 18.0  # Threat detection range
```

## Simulation Phases

### Phase 1: Individual Search (SEARCH)
- Agents explore randomly with individual velocities
- No coordination or communication
- **Transition:** When agents encounter neighbors

### Phase 2: Group Formation (GROUP)
- Flocking behavior with alignment and cohesion
- Collective movement toward common direction
- **Transition:** When group achieves connectivity and velocity clustering

### Phase 3: V-Formation Flight (LINE)
- Structured formation with leader selection
- Velocity locking mechanism for synchronized flight
- **Transition:** When formation is stable and locked

### Phase 4: Threat Response (RING)
- Coordinated encirclement of detected threat
- Ring formation with angular spacing
- **Success:** When all agents achieve NEUTRALIZED status

## Understanding the Output

### Visual Mode
The animation shows agents colored by their current behavioral mode:
- 🔵 **Blue:** SEARCH (individual exploration)
- 🟡 **Yellow:** GROUP (collective movement)
- 🟠 **Orange:** LINE (V-formation)
- 🔴 **Red:** RING (threat engagement)
- 🟢 **Green:** NEUTRALIZED (mission complete)

The threat appears as a red circle that turns green when successfully neutralized.

### Validation Mode Output
```
V-formation achieved at frame 156
Threat activated at frame 278
Neutralization completed at frame 445
Settlement achieved at frame 467
Total simulation frames 1000
SUCCESS: Mission completed - threat neutralized and agents settled
```

### Batch Analysis Results
```
ENHANCED SWARM SIMULATION BATCH ANALYSIS
========================================

OVERALL PERFORMANCE:
  Total simulations run: 1000
  Successful missions: 847
  Success rate: 84.7%
  Status: EXCELLENT - High reliability swarm behavior

TOP 10 SEEDS - BEST V-FORMATION QUALITY:
  Seed   Quality  V-Form Frame Total Frames
  42     89.2     108          432
  123    87.5     125          445
  ...

OPTIMAL SEEDS FOR DIFFERENT PURPOSES:
  - Best V-formation demos: [42, 123, 234, 345, 456]
  - Fastest mission completion: [789, 890, 901, 12, 234]
```

## Performance Metrics

### V-Formation Quality
Measures how quickly and efficiently agents achieve coordinated flight:
- **Score:** 0-100% (higher is better)
- **Calculation:** Based on frames to achieve LINE mode
- **Best Seeds:** Those achieving formation in <150 frames

### Mission Efficiency  
Measures overall mission completion speed:
- **Score:** 0-100% (higher is better)
- **Calculation:** Based on total frames to settlement
- **Best Seeds:** Those completing mission in <500 frames

## Troubleshooting

### Common Issues

**Low Success Rate (<50%)**
- Adjust `V_FORMATION` or `RING` parameters in settings
- Check `THREAT_DELAY` timing
- Verify `SENSING_RADIUS` values

**Agents Not Forming Groups**
- Increase `SENSING_RADIUS` or `SIGHT_RADIUS`
- Adjust `COHESION_WEIGHT` and `SEPARATION_WEIGHT`

**V-Formation Not Locking**
- Check `V_LOCK_DELAY_FRAMES` setting
- Verify `V_ATTRACTION_WEIGHT` and `V_REPULSION_WEIGHT`

**Threat Never Appears**
- Ensure agents reach LINE mode successfully
- Check `THREAT_DELAY` isn't too long for simulation duration
- Verify `is_v_formation_locked()` conditions

### Debugging Specific Seeds

```bash
# Visualize a problematic seed
python simulation.py --seed 999

# Get detailed timing for failed seed
python simulation.py --seed 999 --no-visual
```

## Advanced Usage

### Parameter Tuning
1. Identify failed seeds using batch tester
2. Visualize specific failures with `--seed`
3. Adjust relevant settings parameters
4. Re-test with batch analysis

### Custom Analysis
The batch tester saves detailed results to JSON files for custom analysis:
```python
import json
with open('detailed_results_1_1000.json', 'r') as f:
    data = json.load(f)
    # Custom analysis of timing patterns, failure modes, etc.
```

### Integration Testing
```bash
# Test parameter changes across seed range
python batch_tester.py --start 1 --end 100
# Modify settings.py
python batch_tester.py --start 1 --end 100
# Compare results
```

## Requirements

- Python 3.7+
- NumPy
- Matplotlib
- Standard library modules (math, random, collections, subprocess, json)

## License

This project is developed for educational and research purposes in swarm intelligence and multi-agent systems.
