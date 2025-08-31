import math
import random
from collections import deque
import settings


class Agent:
    """
    Represents an individual agent in a swarm simulation with multiple behavioral modes:
    SEARCH, GROUP, LINE (V-formation), RING, and NEUTRALIZED.
    """

    def __init__(self, agent_id, position, heading):
        """
        Initialize a new agent with basic properties and deterministic search behavior.

        Args:
            agent_id: Unique identifier for the agent
            position: Initial (x, y) coordinates
            heading: Initial direction in radians
        """
        self.id = agent_id
        self.position = position
        self.heading = heading
        self.velocity = (0.0, 0.0)
        self.mode = 'SEARCH'
        self.threat_pos = None
        self.neighbors = []
        self.history = deque(maxlen=3)  # Velocity smoothing buffer

        # V-formation specific attributes
        self.v_locked = False
        self.v_lock_timer = 0
        self.lock_signal = False
        self.group_size = 1
        self.leader = None

        # Create deterministic random state for this agent
        agent_seed = hash((agent_id, position[0], position[1], heading)) % (2 ** 31)
        self.random_state = random.Random(agent_seed)

        # Initialize deterministic search velocity
        search_speed = settings.MAX_SPEED * self.random_state.uniform(0.1, 0.75)
        self.search_velocity = (
            math.cos(self.heading) * search_speed,
            math.sin(self.heading) * search_speed
        )

    def update(self, agents, threat_info):
        """
        Main update loop for agent behavior and position.
        Handles mode transitions and computes movement based on current mode.

        Args:
            agents: List of all agents in the simulation
            threat_info: Tuple of (active, threat_position)
        """
        active, threat_pos = threat_info

        # Find nearby neighbors within sensing radius
        self.neighbors = [
            ag for ag in agents
            if ag is not self and self._is_within_radius(ag.position, settings.SENSING_RADIUS)
        ]

        # Update group size for GROUP and LINE modes
        self.group_size = 1 + sum(1 for nbr in self.neighbors if nbr.mode in ('GROUP', 'LINE'))

        # Mode transition logic: SEARCH -> GROUP -> LINE -> RING -> NEUTRALIZED

        # Threat detection: transition to RING mode
        if active and threat_pos and self._is_within_radius(threat_pos, settings.THREAT_DETECTION_RADIUS):
            self.mode = 'RING'
            self.threat_pos = threat_pos

        # Propagate RING mode from neighbors
        for nbr in self.neighbors:
            if nbr.mode == 'RING':
                self.mode = 'RING'
                self.threat_pos = nbr.threat_pos
                break

        # Transition from SEARCH to GROUP when meeting other agents
        if self.mode == 'SEARCH':
            for nbr in self.neighbors:
                if nbr.mode in ('SEARCH', 'GROUP', 'LINE'):
                    self.mode = 'GROUP'
                    break

        # Convert nearby SEARCH agents to GROUP mode
        if self.mode == 'GROUP':
            for nbr in self.neighbors:
                if nbr.mode == 'SEARCH':
                    nbr.mode = 'GROUP'

        # Compute velocity based on current mode
        if self.mode == 'SEARCH':
            vx, vy = self.search_velocity
        elif self.mode == 'GROUP':
            vx, vy = self._compute_group()
        elif self.mode == 'LINE':
            vx, vy = self._compute_v_formation(agents)
        elif self.mode == 'RING':
            vx, vy = self._compute_ring()
        else:
            vx, vy = 0.0, 0.0

        # Apply velocity smoothing using history buffer
        self.history.append((vx, vy))
        vx = sum(v[0] for v in self.history) / len(self.history)
        vy = sum(v[1] for v in self.history) / len(self.history)

        # Enforce maximum speed constraint
        speed = math.hypot(vx, vy)
        if speed > settings.MAX_SPEED:
            factor = settings.MAX_SPEED / speed
            vx *= factor
            vy *= factor
        self.velocity = (vx, vy)

        # Update position with toroidal world wrapping
        x = (self.position[0] + vx * settings.TIME_STEP) % settings.WORLD_WIDTH
        y = (self.position[1] + vy * settings.TIME_STEP) % settings.WORLD_HEIGHT
        self.position = (x, y)

        # Update heading based on movement direction
        if speed > 1e-6:
            self.heading = math.atan2(vy, vx)

        # Update group size for connected agents
        if self.mode in ('GROUP', 'LINE'):
            self.group_size = 1 + sum(1 for nbr in self.neighbors if nbr.mode in ('GROUP', 'LINE'))
        else:
            self.group_size = 1

        # Check for threat neutralization in RING mode
        if self.mode == 'RING':
            errs = []
            for ag in ([self] + self.neighbors):
                if ag.mode == 'RING':
                    dx = ag.position[0] - self.threat_pos[0]
                    dy = ag.position[1] - self.threat_pos[1]
                    errs.append(abs(math.hypot(dx, dy) - settings.RING_RADIUS))
            if errs and max(errs) < settings.RING_NEUTRALIZE_TOLERANCE:
                self.mode = 'NEUTRALIZED'

    def _compute_group(self):
        """
        Compute velocity for GROUP mode using weighted heading averaging.
        Agents align their movement direction with nearby group members.
        """
        sin_sum = math.sin(self.heading) * self.group_size
        cos_sum = math.cos(self.heading) * self.group_size
        total_weight = self.group_size

        # Weight neighbor headings by their group sizes
        for nbr in self.neighbors:
            if nbr.mode in ('GROUP', 'LINE'):
                sin_sum += math.sin(nbr.heading) * nbr.group_size
                cos_sum += math.cos(nbr.heading) * nbr.group_size
                total_weight += nbr.group_size

        # Calculate average heading direction
        if total_weight > 0:
            avg_heading = math.atan2(sin_sum, cos_sum)
        else:
            avg_heading = self.heading

        speed = settings.MAX_SPEED * 0.5
        return math.cos(avg_heading) * speed, math.sin(avg_heading) * speed

    def _compute_v_formation(self, agents):
        """
        Compute velocity for LINE mode (V-formation flight).
        Implements a locking mechanism for synchronized formation flight.
        """
        # Initialize V-formation state variables if not present
        if not hasattr(self, "v_locked"):              self.v_locked = False
        if not hasattr(self, "v_lock_timer"):          self.v_lock_timer = 0
        if not hasattr(self, "lock_signal"):           self.lock_signal = False
        if not hasattr(self, "lock_signal_timer"):     self.lock_signal_timer = 0
        if not hasattr(self, "locked_velocity"):       self.locked_velocity = (0.0, 0.0)

        # V-formation parameters from settings
        cruise_speed = 0.5 * settings.MAX_SPEED
        R = settings.V_DISTANCE
        lock_threshold = 0.9 * R
        wing_corridor = 0.33 * R
        LOCK_DELAY_FRAMES = settings.V_LOCK_DELAY_FRAMES
        SIGNAL_FRAMES = 6

        # Calculate formation geometry based on leader
        heading = self.leader.heading
        fdx, fdy = math.cos(heading), math.sin(heading)
        leader_pos = self.leader.position
        leader_locked_v = (fdx * cruise_speed, fdy * cruise_speed)

        # Define wing directions (left and right sides of V)
        left_ang = heading + settings.V_SPREAD_ANGLE
        right_ang = heading - settings.V_SPREAD_ANGLE
        wLx, wLy = -math.cos(left_ang), -math.sin(left_ang)
        wRx, wRy = -math.cos(right_ang), -math.sin(right_ang)

        def perp_dist_to_line(px, py, ux, uy):
            """Calculate perpendicular distance from point to wing line."""
            dx = self._wrapped_dx(px, leader_pos[0])
            dy = self._wrapped_dy(py, leader_pos[1])
            v2 = dx * dx + dy * dy
            dot = dx * ux + dy * uy
            return max(0.0, (v2 - dot * dot)) ** 0.5

        # Find agents in LINE mode within local neighborhood
        line_neighbors = [n for n in self.neighbors if n.mode == 'LINE']
        if self.mode == 'LINE' and self not in line_neighbors:
            line_neighbors.append(self)

        def is_local_leader():
            """Determine if this agent is the local leader (lowest ID)."""
            if not line_neighbors:
                return False
            return self.id == min(a.id for a in line_neighbors)

        # Leader decision making and signal emission
        if is_local_leader():
            sliding_max_dist, any_sliding = 0.0, False
            lx, ly = leader_pos

            # Check if agents are properly positioned in wing corridors
            for a in line_neighbors:
                ax, ay = a.position
                d_perp = min(
                    perp_dist_to_line(ax, ay, wLx, wLy),
                    perp_dist_to_line(ax, ay, wRx, wRy)
                )
                sliding = d_perp > wing_corridor
                dx = self._wrapped_dx(ax, lx)
                dy = self._wrapped_dy(ay, ly)
                d_leader = math.hypot(dx, dy)
                if sliding:
                    any_sliding = True
                    if d_leader > sliding_max_dist:
                        sliding_max_dist = d_leader

            # Lock formation when agents are properly aligned
            ready_to_lock = (not any_sliding) or (sliding_max_dist <= lock_threshold)
            if ready_to_lock:
                self.v_locked = True
                self.lock_signal = True
                self.lock_signal_timer = max(self.lock_signal_timer, SIGNAL_FRAMES)
                self.locked_velocity = leader_locked_v

        # Lock signal propagation (wave-like spreading)
        if not self.v_locked:
            source = None
            for n in line_neighbors:
                if getattr(n, "v_locked", False) or getattr(n, "lock_signal", False):
                    lvx, lvy = getattr(n, "locked_velocity", (0.0, 0.0))
                    if (lvx * lvx + lvy * lvy) > 1e-9:
                        source = n
                        break

            if source is not None:
                if self.v_lock_timer == 0:
                    self.v_lock_timer = LOCK_DELAY_FRAMES
                    self.locked_velocity = source.locked_velocity

            # Countdown to lock activation
            if self.v_lock_timer > 0:
                self.v_lock_timer -= 1
                if self.v_lock_timer == 0:
                    self.v_locked = True
                    self.lock_signal = True
                    self.lock_signal_timer = max(self.lock_signal_timer, SIGNAL_FRAMES)
                    # Ensure valid locked velocity
                    lvx, lvy = self.locked_velocity
                    if (lvx * lvx + lvy * lvy) <= 1e-9:
                        self.locked_velocity = leader_locked_v

        # Signal decay timer
        if self.lock_signal_timer > 0:
            self.lock_signal_timer -= 1
            if self.lock_signal_timer == 0:
                self.lock_signal = False

        # Locked agents move rigidly with leader's velocity
        if self.v_locked:
            vx, vy = self.locked_velocity
            n = math.hypot(vx, vy)
            if n > 1e-9:
                s = cruise_speed / n
                return vx * s, vy * s
            else:
                return leader_locked_v

        # Sliding dynamics for unlocked agents
        if self.id == self.leader.id:
            return leader_locked_v

        # Calculate movement towards optimal wing position
        dx = self._wrapped_dx(self.position[0], leader_pos[0])
        dy = self._wrapped_dy(self.position[1], leader_pos[1])
        dist_to_leader = math.hypot(dx, dy)

        def project_to_wing(target_angle):
            """Project agent to wing position at same distance from leader."""
            wx = leader_pos[0] - math.cos(target_angle) * dist_to_leader
            wy = leader_pos[1] - math.sin(target_angle) * dist_to_leader
            return wx, wy

        # Find closest wing position (left or right)
        left_target = project_to_wing(left_ang)
        right_target = project_to_wing(right_ang)

        dx_left = self._wrapped_dx(left_target[0], self.position[0])
        dy_left = self._wrapped_dy(left_target[1], self.position[1])
        dist_left = math.hypot(dx_left, dy_left)

        dx_right = self._wrapped_dx(right_target[0], self.position[0])
        dy_right = self._wrapped_dy(right_target[1], self.position[1])
        dist_right = math.hypot(dx_right, dy_right)

        # Choose closer wing and calculate attraction force
        if dist_left < dist_right:
            attract_x, attract_y = dx_left, dy_left
        else:
            attract_x, attract_y = dx_right, dy_right

        # Normalize attraction vector
        norm = math.hypot(attract_x, attract_y)
        if norm > 1e-6:
            attract_x /= norm
            attract_y /= norm

        # Calculate local repulsion to avoid collisions
        repx = repy = 0.0
        for nbr in self.neighbors:
            if nbr is self:
                continue
            ndx = self._wrapped_dx(self.position[0], nbr.position[0])
            ndy = self._wrapped_dy(self.position[1], nbr.position[1])
            d = math.hypot(ndx, ndy)
            if 0 < d < settings.V_REPULSION_RADIUS:
                strength = (settings.V_REPULSION_RADIUS - d) / settings.V_REPULSION_RADIUS
                repx += (ndx / d) * strength
                repy += (ndy / d) * strength

        # Combine attraction and repulsion, scale to cruise speed
        vx = attract_x * settings.V_ATTRACTION_WEIGHT + repx * settings.V_REPULSION_WEIGHT
        vy = attract_y * settings.V_ATTRACTION_WEIGHT + repy * settings.V_REPULSION_WEIGHT
        sp = math.hypot(vx, vy)
        if sp > 1e-6:
            s = cruise_speed / sp
            vx *= s
            vy *= s
        else:
            vx = vy = 0.0

        return vx, vy

    def _wrapped_dx(self, x1, x2):
        """Calculate x-distance with toroidal world wrapping."""
        dx = x1 - x2
        if abs(dx) > settings.WORLD_WIDTH / 2:
            if dx > 0:
                dx -= settings.WORLD_WIDTH
            else:
                dx += settings.WORLD_WIDTH
        return dx

    def _wrapped_dy(self, y1, y2):
        """Calculate y-distance with toroidal world wrapping."""
        dy = y1 - y2
        if abs(dy) > settings.WORLD_HEIGHT / 2:
            if dy > 0:
                dy -= settings.WORLD_HEIGHT
            else:
                dy += settings.WORLD_HEIGHT
        return dy

    def _compute_ring(self):
        """
        Compute velocity for RING mode to encircle and neutralize threats.
        Combines radial positioning and tangential movement with angular spacing.
        """
        tp = self.threat_pos
        if tp is None:
            return 0.0, 0.0

        # Calculate radial error (distance from ideal ring radius)
        dx, dy = self.position[0] - tp[0], self.position[1] - tp[1]
        dist = math.hypot(dx, dy)
        err = dist - settings.RING_RADIUS

        # Radial correction velocity
        if dist > 1e-6:
            rvx = (dx / dist) * (-err)
            rvy = (dy / dist) * (-err)
            # Tangential velocity for circulation
            tvx = -dy / dist
            tvy = dx / dist
        else:
            rvx = rvy = tvx = tvy = 0.0

        # Add tangential movement
        rvx += tvx * settings.RING_TANGENTIAL_SPEED
        rvy += tvy * settings.RING_TANGENTIAL_SPEED

        # Angular spacing repulsion to distribute agents evenly around ring
        repulsion_x = repulsion_y = 0.0
        ideal_angular_spacing = 2 * math.pi / settings.NUM_AGENTS
        repulsion_weight = settings.RING_REPULSION_WEIGHT
        agent_angle = math.atan2(dy, dx)

        for nbr in self.neighbors:
            neighbor_dx, neighbor_dy = nbr.position[0] - tp[0], nbr.position[1] - tp[1]
            neighbor_angle = math.atan2(neighbor_dy, neighbor_dx)
            angular_delta = (agent_angle - neighbor_angle + math.pi) % (2 * math.pi) - math.pi

            if abs(angular_delta) < ideal_angular_spacing:
                overlap = ideal_angular_spacing - abs(angular_delta)
                direction = 1 if angular_delta > 0 else -1
                repulsion_x += repulsion_weight * overlap * direction * tvx
                repulsion_y += repulsion_weight * overlap * direction * tvy

        return rvx + repulsion_x, rvy + repulsion_y

    def _is_within_radius(self, other_pos, radius):
        """Check if another position is within given radius (with world wrapping)."""
        dx = self._wrapped_dx(other_pos[0], self.position[0])
        dy = self._wrapped_dy(other_pos[1], self.position[1])
        return dx * dx + dy * dy <= radius * radius

    def _is_in_full_group(self, agents):
        """Check if all agents are in GROUP or LINE mode."""
        return all(ag.mode in ('GROUP', 'LINE') for ag in agents)

    def _get_leader(self, visible_agents):
        """
        Determine the leader agent based on position relative to movement direction.
        The leader is the agent furthest forward in the direction of group movement.
        """
        if not visible_agents:
            return None

        # Calculate average velocity to determine movement direction
        total_vx = sum(ag.velocity[0] for ag in visible_agents)
        total_vy = sum(ag.velocity[1] for ag in visible_agents)

        avg_velocity_magnitude = math.hypot(total_vx, total_vy)
        if avg_velocity_magnitude < 1e-6:
            # Fallback to heading average if no clear velocity
            sin_sum = sum(math.sin(ag.heading) for ag in visible_agents)
            cos_sum = sum(math.cos(ag.heading) for ag in visible_agents)
            if abs(sin_sum) < 1e-6 and abs(cos_sum) < 1e-6:
                return min(visible_agents, key=lambda a: a.id)
            movement_direction = math.atan2(sin_sum, cos_sum)
            direction_x = math.cos(movement_direction)
            direction_y = math.sin(movement_direction)
        else:
            direction_x = total_vx / avg_velocity_magnitude
            direction_y = total_vy / avg_velocity_magnitude

        # Find agent with maximum projection in movement direction
        best_agent = None
        best_projection = float('-inf')

        for agent in visible_agents:
            # Calculate group centroid
            centroid_x = sum(ag.position[0] for ag in visible_agents) / len(visible_agents)
            centroid_y = sum(ag.position[1] for ag in visible_agents) / len(visible_agents)

            # Project agent position onto movement direction
            rel_x = self._wrapped_dx(agent.position[0], centroid_x)
            rel_y = self._wrapped_dy(agent.position[1], centroid_y)
            projection = rel_x * direction_x + rel_y * direction_y

            if projection > best_projection or (projection == best_projection and
                                                (best_agent is None or agent.id < best_agent.id)):
                best_projection = projection
                best_agent = agent

        return best_agent

    @staticmethod
    def is_all_agents_connected(agents):
        """
        Check if all agents form a connected graph through sensing radius.
        Uses breadth-first search to verify connectivity.
        """
        if not agents:
            return False

        visited = set()
        to_visit = [agents[0]]

        while to_visit:
            current = to_visit.pop()
            if current.id in visited:
                continue
            visited.add(current.id)
            neighbors = [
                other for other in agents
                if
                other.id not in visited and Agent._distance(current.position, other.position) <= settings.SENSING_RADIUS
            ]
            to_visit.extend(neighbors)

        return len(visited) == len(agents)

    def set_line_mode(self, agents):
        """
        Transition agent to LINE mode and establish leader relationship.

        Args:
            agents: All agents in simulation for leader selection
        """
        self.mode = 'LINE'

        visible_agents = [
            ag for ag in agents
            if self._is_within_radius(ag.position, settings.SIGHT_RADIUS)
        ]

        self.leader = self._get_leader(visible_agents)
        settings.has_found_leader()

    @staticmethod
    def _distance(pos1, pos2):
        """Calculate Euclidean distance between two positions."""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return math.hypot(dx, dy)

    @staticmethod
    def _is_velocity_clustered_static(agents, angle_threshold=0.2):
        """
        Check if all agents have similar velocity directions (velocity clustering).
        Used to determine when the group is moving coherently.
        """
        avg_vx = sum(a.velocity[0] for a in agents) / len(agents)
        avg_vy = sum(a.velocity[1] for a in agents) / len(agents)
        norm = math.hypot(avg_vx, avg_vy)
        if norm == 0:
            return False
        avg_vx /= norm
        avg_vy /= norm

        # Check if all individual velocities are within angle threshold of average
        for a in agents:
            vx, vy = a.velocity
            vnorm = math.hypot(vx, vy)
            if vnorm == 0:
                return False
            vx /= vnorm
            vy /= vnorm
            dot = vx * avg_vx + vy * avg_vy
            if dot < math.cos(angle_threshold):
                return False
        return True
