"""
Keep the arm visibly alive while it waits for something to happen.

The listening state needs motion that is interesting to look at and boring to reason
about. This wanders the gripper between random waypoints and twitches the wrist, but it
never invents a new kind of motion: every translation goes through
``xArm.move_relative``, so each one is re-validated against the live pose and the
configured safety envelope before it is sent, exactly like a planned move.

Two further limits are specific to idling. The waypoints are confined to a box around
wherever the arm stood when idling began, so hours of wandering cannot walk the gripper
across the bench to a station. And every individual step is short, so the longest the
arm can still be moving after a soft stop is one step.

    motion = IdleMotion(arm)
    motion.check_ready()          # raises SafetyError with the reason, before anything moves
    motion.start()
    ...
    motion.request_stop()
    motion.join()
"""
import random
import threading

from robotic_testing.common.robotic_arms.xarm_control import SafetyError

# Keep off the envelope walls: a waypoint exactly on the boundary is one rounding error
# away from a rejected move, and rejected moves make the idle look like it is stuttering.
EDGE_MARGIN_MM = 10.0


class IdleMotion:
    """
    Wander the gripper around its starting pose until asked to stop.

    :param radius_mm: half-width of the wander box around the starting pose, in x and y
    :param z_radius_mm: same in z; defaults to half ``radius_mm``, since down is the bench
    :param step_mm: length of a single commanded move; capped by the arm's own step limit
    :param pause_chance: probability that a waypoint is followed by a short pause
    :param wiggle_chance: probability that a waypoint is followed by a wrist twist
    :param wiggle_deg: largest wrist twist away from the starting joint angle
    :param settings_profile: pos_settings_dict key; defaults to the arm's free-form profile
    :param origin: centre of the wander box; defaults to wherever the arm is when idling
        starts. Pass the same origin every time you restart, or the box follows the arm and
        a few stop-and-resume cycles walk it somewhere you did not intend.
    :param seed: fix it to get a repeatable wander, e.g. when demonstrating
    """

    def __init__(self, arm, radius_mm=150.0, z_radius_mm=None, step_mm=40.0,
                 pause_chance=0.25, pause_s=(0.4, 1.8), wiggle_chance=0.35, wiggle_deg=20.0,
                 settings_profile=None, origin=None, seed=None):
        self.arm = arm
        self.radius_mm = float(radius_mm)
        self.z_radius_mm = float(radius_mm / 2 if z_radius_mm is None else z_radius_mm)
        self.step_mm = float(step_mm)
        self.pause_chance = float(pause_chance)
        self.pause_s = tuple(pause_s)
        self.wiggle_chance = float(wiggle_chance)
        self.wiggle_deg = float(wiggle_deg)
        self.settings_profile = settings_profile
        self.origin = list(origin) if origin is not None else None
        self.rng = random.Random(seed)

        self._stop = threading.Event()
        self._thread = None
        self.moves = 0
        self.waypoints = 0
        self.error = None

        self.bounds = None
        self._wrist_joint = None
        self._wrist_start = None

    # ------------------------------------------------------------------
    # preconditions
    # ------------------------------------------------------------------

    def check_ready(self):
        """
        Refuse to idle unless idling is harmless right now.

        Same reasoning as ``xArm.wave``: an arm that is holding something, or that is in
        the middle of a sample workflow, has no business wandering around at random.

        :raises SafetyError: with the reason, before anything has moved
        """
        arm = self.arm
        arm.assert_ready()
        if arm.sample_in_flask:
            raise SafetyError('a sample is still in the flask; finish that workflow before idling')
        if arm.holding_object:
            raise SafetyError('the arm is holding an object; place it down before idling')
        if arm.has_sample_holder:
            raise SafetyError(
                f'the gripper is closed to {arm.gripper_opening:.1f} mm and may be holding the '
                f'sample holder; open it before idling'
            )
        pose = arm.get_cartesian_pos()
        allowed, reason = arm.check_pose_allowed(pose)
        if not allowed:
            raise SafetyError(f'the arm is outside its safety envelope ({reason}); it cannot idle from here')
        self._plan_bounds(pose)
        return True

    def _plan_bounds(self, pose):
        """Intersect the wander box with the configured envelope."""
        safety = self.arm._envelope()
        origin = self.origin if self.origin is not None else list(pose)
        radii = (self.radius_mm, self.radius_mm, self.z_radius_mm)
        bounds = []
        for axis, key in enumerate(('x_range', 'y_range', 'z_range')):
            env_low, env_high = (float(value) for value in safety[key])
            low = max(env_low + EDGE_MARGIN_MM, origin[axis] - radii[axis])
            high = min(env_high - EDGE_MARGIN_MM, origin[axis] + radii[axis])
            if high - low < 2 * EDGE_MARGIN_MM:
                raise SafetyError(
                    f'there is no room to idle along {"xyz"[axis]}: the box is centred on '
                    f'{origin[axis]:.1f} mm, and the envelope only allows '
                    f'{env_low:.1f} .. {env_high:.1f} mm. Move the arm away from the edge '
                    f'first, or widen the envelope in the arm config if the volume really '
                    f'is clear.'
                )
            bounds.append((low, high))

        self.origin = origin
        self.bounds = bounds
        # a step longer than the arm's own single-step limit would be rejected every time
        self.step_mm = min(self.step_mm, float(safety['max_step_mm']))
        if self.wiggle_chance > 0:
            self._wrist_joint = int(safety['wave']['joint'] or self.arm.num_joints)
            self._wrist_start = self.arm.get_joint_angles()[self._wrist_joint - 1]
        return bounds

    def describe(self):
        if self.bounds is None:
            return 'idle motion has not worked out its limits yet'
        return '\n'.join([
            'Idling inside:',
            '  x: {:.1f} .. {:.1f} mm'.format(*self.bounds[0]),
            '  y: {:.1f} .. {:.1f} mm'.format(*self.bounds[1]),
            '  z: {:.1f} .. {:.1f} mm'.format(*self.bounds[2]),
            f'  steps of up to {self.step_mm:.0f} mm'
            + (f', wrist joint {self._wrist_joint} twisting up to {self.wiggle_deg:.0f} deg'
               if self.wiggle_chance > 0 else ''),
        ])

    # ------------------------------------------------------------------
    # running
    # ------------------------------------------------------------------

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        """Begin wandering on a background thread. Call :meth:`check_ready` first."""
        if self.running:
            raise RuntimeError('idle motion is already running')
        if self.bounds is None:
            self.check_ready()
        self._stop.clear()
        self.error = None
        self._thread = threading.Thread(target=self._run, name='idle-motion', daemon=True)
        self._thread.start()
        return self

    def request_stop(self):
        """
        Ask the wander loop to finish.

        This on its own is a *soft* stop: the move already in flight runs to its end, so
        the arm keeps moving for up to one step. Halting the arm mid-step is the job of
        ``xArm.emergency_stop``, which the caller invokes alongside this.
        """
        self._stop.set()

    def join(self, timeout=10.0):
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        return not self.running

    def _sleep(self, seconds):
        """Interruptible pause. Returns False if a stop was requested during it."""
        return not self._stop.wait(seconds)

    def _run(self):
        rejections = 0
        try:
            while not self._stop.is_set():
                try:
                    self._to_waypoint()
                except SafetyError as error:
                    if self._stop.is_set():
                        return
                    try:
                        # a stop landed mid-move: that is the end of the session, not a
                        # waypoint to retry
                        self.arm.assert_ready()
                    except SafetyError as fault:
                        self.error = fault
                        return
                    # a waypoint the arm cannot actually reach is expected now and then --
                    # the envelope is a box, the arm's reachable volume is not -- so try a
                    # different one rather than ending the session
                    rejections += 1
                    if rejections > 10:
                        self.error = SafetyError(
                            f'gave up idling after 10 rejected moves in a row; last reason: {error}'
                        )
                        return
                    continue
                rejections = 0
        except Exception as error:
            if not self._stop.is_set():
                self.error = error
        finally:
            self._stop.set()

    def _pick_waypoint(self):
        """
        A random point in the box that the envelope will actually accept.

        The box is axis-aligned but the envelope is not only a box -- the arm configs also
        keep a cylindrical exclusion around the base column and a maximum reach. Checking
        the waypoint here costs nothing and saves walking towards one that the arm will
        refuse the last step of.
        """
        for _ in range(20):
            candidate = [self.rng.uniform(low, high) for low, high in self.bounds]
            allowed, _ = self.arm.check_pose_allowed(candidate)
            if allowed:
                return candidate
        raise SafetyError(
            'could not find a reachable point to wander to in 20 tries; the idle box barely '
            'overlaps what the envelope allows, so try a smaller --radius or move the arm'
        )

    def _to_waypoint(self):
        """Walk to one random waypoint in short validated steps, then maybe pause or twist."""
        target = self._pick_waypoint()
        self.waypoints += 1

        while not self._stop.is_set():
            pose = self.arm.get_cartesian_pos()
            delta = [target[axis] - pose[axis] for axis in range(3)]
            distance = sum(value ** 2 for value in delta) ** 0.5
            if distance < self.step_mm:
                break
            scale = self.step_mm / distance
            self.arm.move_relative(*[value * scale for value in delta],
                                   settings_profile=self.settings_profile)
            self.moves += 1

        if self._stop.is_set():
            return
        if self.wiggle_chance and self.rng.random() < self.wiggle_chance:
            self._wiggle()
        if self.pause_chance and self.rng.random() < self.pause_chance:
            self._sleep(self.rng.uniform(*self.pause_s))

    def _wiggle(self):
        """
        Twist the wrist a little, without moving the gripper anywhere.

        The angle is always chosen relative to the joint angle recorded when idling
        began, so however long this runs the wrist cannot creep towards its limit.
        """
        safety = self.arm.safety['wave']
        limit = abs(float(safety['joint_limit_deg']))
        angle = self._wrist_start + self.rng.uniform(-self.wiggle_deg, self.wiggle_deg)
        angle = max(-limit, min(limit, angle))
        self.arm.set_servo_angle(servo_id=self._wrist_joint, angle=angle,
                                 **self.arm.angle_settings(safety['settings_profile']))
        self.moves += 1
