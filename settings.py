import math

"""
Configuration settings for swarm simulation with progressive behavioral phases:
SEARCH -> GROUP -> LINE (V-formation) -> RING (threat neutralization)
"""

# =============================================================================
# WORLD AND SIMULATION PARAMETERS
# =============================================================================

# World dimensions (meters) - toroidal topology
WORLD_WIDTH = 100.0
WORLD_HEIGHT = 100.0

# Simulation timing control
TIME_STEP = 1.0            # seconds per simulation tick
TOTAL_STEPS = 1000         # total simulation duration
THREAT_DELAY = 120.0       # seconds to wait after V-formation locks before threat appears

# =============================================================================
# AGENT PHYSICAL PARAMETERS
# =============================================================================

NUM_AGENTS = 10            # total swarm size
MAX_SPEED = 2.0           # maximum agent velocity (m/s)
MAX_TURN_RATE = 0.1       # maximum heading change per step (radians)

# Sensing and detection radii
SENSING_RADIUS = 10.0      # local neighbor detection for flocking behaviors
SIGHT_RADIUS = 50.0       # extended visibility for leader selection
THREAT_DETECTION_RADIUS = 18.0  # range for detecting threats

# =============================================================================
# GROUP FORMATION PARAMETERS  
# =============================================================================

# Migration behavior for GROUP mode
MIGRATION_DIRECTION = 0.0      # global migration direction (set at runtime)
MIGRATION_BIAS_WEIGHT = 10.0   # strength of directional bias

# Flocking forces
SEPARATION_RADIUS = 10.0
COHESION_RADIUS = 25.0
SEPARATION_WEIGHT = 0.5
COHESION_WEIGHT = 0.05

# =============================================================================
# V-FORMATION (LINE MODE) PARAMETERS
# =============================================================================

# V-formation geometry
V_SPREAD_ANGLE = math.radians(45)  # half-angle of V-formation wings
V_DISTANCE = 25.0                  # spacing between agents on wing lines
V_WING_CORRIDOR = 5.0             # tolerance corridor around wing lines

# V-formation forces and control
V_ATTRACTION_WEIGHT = 5.0     # strength of attraction to wing positions
V_REPULSION_WEIGHT = 2.5      # strength of local collision avoidance
V_REPULSION_RADIUS = 12.0     # radius for local repulsion forces

# Velocity locking mechanism for synchronized flight
V_LOCK_DELAY_FRAMES = 5       # frames to wait before locking to formation velocity
V_LOCK_SIGNAL_FRAMES = 6      # frames to broadcast lock signal to neighbors

# Leader selection parameters
VISION_HALF_ANGLE = math.radians(75)     # field of view for leader detection
ANCHOR_HEADING_COS_MIN = 0.5             # minimum heading similarity for leadership

# =============================================================================
# RING FORMATION PARAMETERS (THREAT RESPONSE)
# =============================================================================

RING_RADIUS = 10.0                    # target radius for threat encirclement
RING_TANGENTIAL_SPEED = 0            # circulation speed around threat
RING_REPULSION_WEIGHT = 7.5          # angular spacing force strength
RING_NEUTRALIZE_TOLERANCE = 0.5      # max distance error for successful neutralization
THREAT_APPEARANCE_TIME = 200

# =============================================================================
# VISUALIZATION PARAMETERS
# =============================================================================

FIG_SIZE = (8, 8)              # matplotlib figure dimensions (inches)
ANIMATION_INTERVAL = 50        # milliseconds between animation frames

# =============================================================================
# ALGORITHM CONTROL PARAMETERS
# =============================================================================

# Convergence and stability thresholds
LINE_STABLE_DELAY = 50             # frames to maintain LINE mode before considering stable
LINE_ALIGNMENT_TOLERANCE = 15      # tolerance for LINE formation alignment

# Communication and connectivity
MAX_COMM_HOPS = NUM_AGENTS         # maximum hops for information propagation
GROUP_JOIN_RADIUS = SENSING_RADIUS # radius for SEARCH->GROUP transitions
LEADER_FOUND = False               # has found leader during iteration

# Randomness control
RNG_SEED = 1285                    # seed for reproducible simulations
