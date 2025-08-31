import random
import settings


class Environment:
    """
    Simulation environment managing world boundaries, threat placement, and deterministic behavior.
    Handles toroidal world topology and threat lifecycle management.
    """

    def __init__(self, agents, seed=None):
        """
        Initialize simulation environment with deterministic random behavior.

        Args:
            agents: List of agents in the simulation
            seed: Random seed for reproducible threat placement (uses RNG_SEED if None)
        """
        self.width = settings.WORLD_WIDTH
        self.height = settings.WORLD_HEIGHT
        self.threat_time = settings.THREAT_APPEARANCE_TIME
        self.threat_position = None
        self.threat_active = False
        self.current_time = 0
        self.agents = agents
        self.stable_counter = 0

        # Use provided seed, fall back to settings, or create deterministic default
        if seed is not None:
            self.random_state = random.Random(seed)
        elif settings.RNG_SEED is not None:
            self.random_state = random.Random(settings.RNG_SEED)
        else:
            # Use deterministic seed based on simulation parameters for consistency
            default_seed = hash((settings.NUM_AGENTS, settings.WORLD_WIDTH, settings.WORLD_HEIGHT)) % (2 ** 31)
            self.random_state = random.Random(default_seed)

    def place_threat(self):
        """
        Place threat at deterministic random position within valid ring formation bounds.

        Ensures the threat appears where agents can form a complete ring without
        exceeding world boundaries. Uses instance random state for reproducibility.
        """
        ring_buffer = settings.RING_RADIUS

        # Calculate valid threat placement area (inset by ring radius)
        min_x = ring_buffer
        max_x = self.width - ring_buffer
        min_y = ring_buffer
        max_y = self.height - ring_buffer

        # Generate deterministic threat position using instance random state
        threat_x = self.random_state.uniform(min_x, max_x)
        threat_y = self.random_state.uniform(min_y, max_y)

        self.threat_position = (threat_x, threat_y)
        self.threat_active = True

    def update(self):
        """
        Update environment state and enforce world topology constraints.
        Applies toroidal boundary conditions to all agents.
        """
        for agent in self.agents:
            self.enforce_boundaries(agent)

    def enforce_boundaries(self, agent):
        """
        Apply toroidal world topology (wrap-around boundaries).

        Agents that move beyond world edges reappear on the opposite side,
        creating a continuous, boundaryless environment.

        Args:
            agent: Agent whose position needs boundary enforcement
        """
        x, y = agent.position
        x %= self.width
        y %= self.height
        agent.position = (x, y)

    def get_threat_info(self):
        """
        Get current threat status and location.

        Returns:
            tuple: (threat_active: bool, threat_position: tuple or None)
        """
        return self.threat_active, self.threat_position
