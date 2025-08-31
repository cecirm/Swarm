import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import settings
from simulation import run_simulation


def animate_swarm(history):
    """
    Create real-time animated visualization of swarm behavioral evolution.

    Visual progression shows complete mission timeline:
    - Blue agents: Individual search and exploration
    - Yellow agents: Group formation and collective movement
    - Orange agents: V-formation flight with velocity locking
    - Red agents: Threat engagement and ring formation
    - Green agents: Successful threat neutralization

    Threat visualization:
    - Red circle: Active threat requiring neutralization
    - Green circle: Successfully neutralized and settled threat

    Args:
        history: Complete simulation data from run_simulation()
    """
    # Initialize matplotlib figure and axes
    fig, ax = plt.subplots(figsize=settings.FIG_SIZE)
    ax.set_xlim(0, settings.WORLD_WIDTH)
    ax.set_ylim(0, settings.WORLD_HEIGHT)
    ax.set_title('Swarm Formation Adaptation')

    # Create visual elements for agents and threat
    agent_scatter = ax.scatter([], [], s=50)
    threat_circle = plt.Circle((0, 0), settings.RING_RADIUS, color='red', alpha=0.3)
    ax.add_patch(threat_circle)

    # Color scheme for different behavioral modes
    mode_colors = {
        'SEARCH': 'blue',  # Individual exploration
        'GROUP': 'yellow',  # Collective movement
        'LINE': 'orange',  # V-formation flight
        'RING': 'red'  # Threat engagement
    }

    # Frame tracking for visual state transitions
    neutralization_frame = None  # When all agents reach NEUTRALIZED mode
    settlement_frame = None  # When agents stop moving after neutralization

    def init():
        """Initialize animation with empty visual elements."""
        agent_scatter.set_offsets(np.empty((0, 2)))
        threat_circle.set_visible(False)
        return agent_scatter, threat_circle

    def update_frame(frame):
        """
        Update single animation frame with current simulation state.

        Handles dynamic color transitions and threat visualization based on
        mission progress and agent settlement detection.

        Args:
            frame: Current animation frame index

        Returns:
            tuple: Updated matplotlib artists for efficient blitting
        """
        nonlocal neutralization_frame, settlement_frame

        # Extract current simulation state
        current_positions = history['positions'][frame]
        current_modes = history['modes'][frame]
        threat_is_active = history['threat'][frame]
        threat_location = history['threat_positions'][frame]

        # Detect neutralization completion (all agents reach NEUTRALIZED state)
        if neutralization_frame is None and all(mode == 'NEUTRALIZED' for mode in current_modes):
            neutralization_frame = frame

        # Detect agent settlement after neutralization (movement stops)
        if neutralization_frame is not None and settlement_frame is None and frame > neutralization_frame:
            previous_positions = history['positions'][frame - 1]

            # Calculate maximum agent displacement between frames
            max_movement = max(
                math.hypot(current_positions[i][0] - previous_positions[i][0],
                           current_positions[i][1] - previous_positions[i][1])
                for i in range(len(current_positions))
            )

            # Define settlement threshold as fraction of maximum possible movement
            settlement_threshold = settings.MAX_SPEED * settings.TIME_STEP * 0.05
            if max_movement < settlement_threshold:
                settlement_frame = frame

        # Update threat circle visualization with state-dependent coloring
        if threat_location:
            threat_circle.center = threat_location
            threat_circle.set_visible(True)
            # Green indicates successful neutralization and settlement
            if settlement_frame is not None and frame >= settlement_frame:
                threat_circle.set_color('green')
            else:
                threat_circle.set_color('red')
        else:
            threat_circle.set_visible(False)

        # Update agent positions on scatter plot
        agent_scatter.set_offsets(np.array(current_positions))

        # Apply color scheme based on behavioral mode and mission state
        agent_colors = []
        for mode in current_modes:
            if settlement_frame is not None and frame >= settlement_frame:
                # Victory state: all agents turn green when mission complete
                agent_colors.append('green')
            elif mode in ('RING', 'NEUTRALIZED'):
                # Active threat response: red during engagement phase
                agent_colors.append('red')
            else:
                # Normal behavioral progression: use mode-specific colors
                agent_colors.append(mode_colors.get(mode, 'gray'))

        # Apply colors to scatter plot with RGBA conversion
        agent_scatter.set_facecolors([mcolors.to_rgba(color) for color in agent_colors])

        return agent_scatter, threat_circle

    # Create matplotlib animation with frame-by-frame updates
    animation_obj = animation.FuncAnimation(
        fig, update_frame,
        frames=len(history['positions']),
        init_func=init,
        interval=settings.ANIMATION_INTERVAL,
        blit=True  # Enable blitting for better performance
    )

    plt.show()


if __name__ == '__main__':
    """
    Execute simulation and display animated visualization.
    """
    simulation_history = run_simulation()
    animate_swarm(simulation_history)