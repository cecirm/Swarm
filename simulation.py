import math
import random
import sys
import argparse
import settings
from env.environment import Environment
from agents.agent import Agent


def is_swarm_ready_for_v_formation(agents, angle_threshold=0.2):
    """
    Determine if swarm meets all criteria for V-formation transition.

    Convergence requirements:
    - Network connectivity (all agents can communicate)
    - Velocity clustering (aligned movement directions)
    - Unified GROUP mode across all agents

    Args:
        agents: List of all agents in simulation
        angle_threshold: Maximum angular deviation for velocity alignment (radians)

    Returns:
        bool: True if ready for LINE mode transition
    """
    if not Agent.is_all_agents_connected(agents):
        return False
    if not Agent._is_velocity_clustered_static(agents, angle_threshold):
        return False
    return all(ag.mode == 'GROUP' for ag in agents)


def is_v_formation_locked(agents):
    """
    Check if V-formation is fully synchronized with locked velocities.
    Required condition for threat activation.

    Args:
        agents: List of all agents in simulation

    Returns:
        bool: True if all agents are in locked LINE formation
    """
    if not agents:
        return False
    return all(a.mode == 'LINE' for a in agents) and all(getattr(a, "v_locked", False) for a in agents)


def is_mission_successful(history):
    """
    Determine if the swarm mission was completed successfully and extract timing metrics.

    Success criteria:
    - Threat was activated at some point
    - All agents reached NEUTRALIZED mode
    - Agents settled (stopped moving) after neutralization

    Args:
        history: Complete simulation history

    Returns:
        tuple: (success: bool, metrics: dict) with timing information
    """
    metrics = {
        'v_formation_frame': None,
        'v_lock_frame': None,
        'threat_activation_frame': None,
        'neutralization_frame': None,
        'settlement_frame': None,
        'total_frames': len(history['positions'])
    }

    if not any(history['threat']):
        return False, metrics  # No threat was ever activated

    # Find key transition frames
    for frame, modes in enumerate(history['modes']):
        # V-formation achievement (first time all agents in LINE mode)
        if metrics['v_formation_frame'] is None and all(mode == 'LINE' for mode in modes):
            metrics['v_formation_frame'] = frame

        # Neutralization completion (all agents reach NEUTRALIZED)
        if metrics['neutralization_frame'] is None and all(mode == 'NEUTRALIZED' for mode in modes):
            metrics['neutralization_frame'] = frame

    # Find threat activation frame
    for frame, threat_active in enumerate(history['threat']):
        if threat_active:
            metrics['threat_activation_frame'] = frame
            break

    if metrics['neutralization_frame'] is None:
        return False, metrics  # Never fully neutralized

    # Check for settlement after neutralization (stopped moving)
    for frame in range(metrics['neutralization_frame'] + 1, len(history['positions'])):
        if frame == 0:
            continue

        current_positions = history['positions'][frame]
        previous_positions = history['positions'][frame - 1]

        # Calculate maximum displacement between frames
        max_movement = max(
            math.hypot(current_positions[i][0] - previous_positions[i][0],
                       current_positions[i][1] - previous_positions[i][1])
            for i in range(len(current_positions))
        )

        # Define settlement threshold
        movement_threshold = settings.MAX_SPEED * settings.TIME_STEP * 0.05
        if max_movement < movement_threshold:
            metrics['settlement_frame'] = frame
            return True, metrics  # Successfully settled after neutralization

    return False, metrics  # Never settled properly


def run_simulation(seed=None):
    """
    Execute complete swarm simulation with progressive behavioral evolution.

    Simulation timeline:
    1. SEARCH: Individual random exploration
    2. GROUP: Collective movement and alignment
    3. LINE: V-formation with velocity synchronization
    4. RING: Coordinated threat encirclement
    5. NEUTRALIZED: Mission completion

    Args:
        seed: Optional random seed for reproducible results (uses RNG_SEED if None)

    Returns:
        dict: Complete simulation history containing:
            - positions: Agent coordinates at each timestep
            - modes: Behavioral states at each timestep
            - threat: Threat activation timeline
            - threat_positions: Threat locations when active
    """
    # Use provided seed, settings seed, or create deterministic default
    if seed is not None:
        simulation_seed = seed
    elif settings.RNG_SEED is not None:
        simulation_seed = settings.RNG_SEED
    else:
        # Generate deterministic seed from simulation parameters
        simulation_seed = hash((settings.NUM_AGENTS, settings.WORLD_WIDTH, settings.WORLD_HEIGHT)) % (2 ** 31)

    random.seed(simulation_seed)

    # Initialize swarm with deterministic random spatial distribution
    agents = [
        Agent(i,
              (random.uniform(0, settings.WORLD_WIDTH),
               random.uniform(0, settings.WORLD_HEIGHT)),
              random.uniform(0, 2 * math.pi))
        for i in range(settings.NUM_AGENTS)
    ]

    # Create simulation environment with same seed for consistency
    env = Environment(agents, simulation_seed)

    # Initialize data collection for analysis and visualization
    history = {
        'positions': [],
        'modes': [],
        'threat': [],
        'threat_positions': []
    }

    # Phase transition control variables
    convergence_timer = 0.0
    threat_ready_timer = 0.0
    line_formation_initiated = False

    # Main simulation loop
    for step in range(settings.TOTAL_STEPS):
        env.update()

        # Get potential threat information (may not be active yet)
        threat_exists, threat_location = env.get_threat_info()

        # Constrain threat to valid ring formation area
        if threat_exists and threat_location:
            radius_buffer = settings.RING_RADIUS
            x, y = threat_location
            x = min(max(x, radius_buffer), settings.WORLD_WIDTH - radius_buffer)
            y = min(max(y, radius_buffer), settings.WORLD_HEIGHT - radius_buffer)
            threat_location = (x, y)

        # Phase 1: Initial agent updates without threat influence
        for agent in agents:
            agent.update(agents, (False, None))

        # Phase transition management: GROUP -> LINE formation
        if not line_formation_initiated:
            if is_swarm_ready_for_v_formation(agents):
                convergence_timer += settings.TIME_STEP
                # Require sustained convergence to prevent premature transitions
                if convergence_timer >= 10.0:
                    for agent in agents:
                        agent.set_line_mode(agents)
                    line_formation_initiated = True
            else:
                convergence_timer = 0.0

        # Threat activation control: wait for stable V-formation
        if not env.threat_active:
            if is_v_formation_locked(agents):
                threat_ready_timer += settings.TIME_STEP
                # Activate threat only after formation stability period
                if threat_ready_timer >= settings.THREAT_DELAY:
                    env.place_threat()
            else:
                threat_ready_timer = 0.0

        # Get final threat state after potential activation
        threat_active, threat_position = env.get_threat_info()

        # Phase 2: Agent updates with active threat information
        for agent in agents:
            agent.update(agents, (threat_active, threat_position))

        # Record complete simulation state
        history['positions'].append([agent.position for agent in agents])
        history['modes'].append([agent.mode for agent in agents])
        history['threat'].append(threat_active)
        history['threat_positions'].append(threat_position if threat_active else None)

    return history


def main():
    """
    Main program entry point with command line argument parsing.
    Supports visual and non-visual (validation) modes.
    """
    parser = argparse.ArgumentParser(description='Swarm Formation Simulation')
    parser.add_argument('--seed', type=int, help='Random seed for reproducible simulations')
    parser.add_argument('--no-visual', action='store_true',
                        help='Run in validation mode (no visualization, exit with success/error code)')

    args = parser.parse_args()

    # Execute simulation
    print("Running swarm simulation...")
    simulation_history = run_simulation(args.seed)

    if args.no_visual:
        # Validation mode: check mission success and exit with appropriate code
        success, metrics = is_mission_successful(simulation_history)

        # Print detailed timing metrics for batch analysis
        if metrics['v_formation_frame'] is not None:
            print(f"V-formation achieved at frame {metrics['v_formation_frame']}")
        if metrics['threat_activation_frame'] is not None:
            print(f"Threat activated at frame {metrics['threat_activation_frame']}")
        if metrics['neutralization_frame'] is not None:
            print(f"Neutralization completed at frame {metrics['neutralization_frame']}")
        if metrics['settlement_frame'] is not None:
            print(f"Settlement achieved at frame {metrics['settlement_frame']}")
        print(f"Total simulation frames {metrics['total_frames']}")

        if success:
            print("SUCCESS: Mission completed - threat neutralized and agents settled")
            sys.exit(0)  # Success exit code
        else:
            print("FAILURE: Mission not completed - threat not properly neutralized")
            sys.exit(1)  # Error exit code
    else:
        # Visual mode: display animation
        from visualization import animate_swarm
        threat_was_active = any(simulation_history['threat'])
        print(f"Simulation completed. Threat ever active? {threat_was_active}")
        animate_swarm(simulation_history)


if __name__ == '__main__':
    main()