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

**Only a commanded move that the arm will accept is ever sent.** A waypoint is committed
to only once the whole straight-line walk to it has been checked against the envelope and
the controller has agreed it can reach it, because the envelope is a box with the base
keep-out punched out of it and the arm's own reachable volume is neither. A waypoint that
does not survive those checks is simply not used -- nothing moves, and another one is
picked. That happens as a matter of course, so it is never treated as a fault: the wander
keeps going until it is asked to stop, or until the arm itself faults, and only the second
of those ends the thread.
"""
import random
import threading

from robotic_testing.common.robotic_arms.xarm_control import SafetyError

# Keep off the envelope walls: a waypoint exactly on the boundary is one rounding error
# away from a rejected move, and rejected moves make the idle look like it is stuttering.
EDGE_MARGIN_MM = 10.0

# The commanded step is the full step length by construction, so a step sitting exactly on
# the arm's single-step limit is one floating-point rounding away from being refused every
# single time. Stay a hair under it.
STEP_MARGIN_MM = 0.5

# How many random candidates to consider before falling back on the walk back to the
# origin. The box is a box and the reachable volume is not, so some candidates always fail.
WAYPOINT_TRIES = 30

# How long to wait after finding nowhere to go, before looking again. Long enough not to
# spin a core, short enough that the arm picks up again promptly once it can.
RETRY_PAUSE_S = 0.5

# Below this share of the box being usable, say so when describing the idle: the arm will
# wander perfectly well, but it will spend visible time looking, and a smaller radius or a
# different starting pose is almost always what was wanted.
CRAMPED_BOX_FRACTION = 0.2


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
    :param ask_controller: put each candidate waypoint to the controller's own kinematics
        before walking to it, as well as to the configured envelope. This is what keeps a
        pose the arm cannot reach from becoming a fault rather than a waypoint quietly not
        used. Turn it off if a particular controller disagrees with itself and refuses
        poses it then happily moves to -- the envelope and path checks still apply.
    :param seed: fix it to get a repeatable wander, e.g. when demonstrating
    """

    #: how many points to sample when surveying the box in :meth:`check_ready`
    SURVEY_POINTS = 200

    def __init__(self, arm, radius_mm=150.0, z_radius_mm=None, step_mm=40.0,
                 pause_chance=0.25, pause_s=(0.4, 1.8), wiggle_chance=0.35, wiggle_deg=20.0,
                 settings_profile=None, origin=None, ask_controller=True, seed=None):
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
        self.ask_controller = bool(ask_controller)
        self.rng = random.Random(seed)

        self._stop = threading.Event()
        self._thread = None
        self.moves = 0
        self.waypoints = 0
        self.error = None

        # a waypoint that turned out not to be usable. Counted and reported, because a
        # wander that is spending most of its time looking for somewhere to go means the
        # box and the reachable volume barely overlap -- but never fatal.
        self.rejections = 0
        self.last_rejection = None
        self.skipped_wiggles = 0
        # candidates turned down by each of the three checks, which is the only way to
        # tell "the box is in the wrong place" from "the controller refuses everything"
        self.refused = {'envelope': 0, 'path': 0, 'controller': 0}
        # share of the box that turned out to be usable, from the survey in check_ready
        self.usable_fraction = None

        self.bounds = None
        self._wrist_joint = None
        self._wrist_start = None
        self._wrist_limit = None

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
        self._survey_box(pose)
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
        # a step longer than the arm's own single-step limit would be rejected every time,
        # and a step exactly on it would be rejected whenever the arithmetic rounded up
        self.step_mm = min(self.step_mm, max(float(safety['max_step_mm']) - STEP_MARGIN_MM, 1.0))
        if self.wiggle_chance > 0:
            self._wrist_joint = int(safety['wave']['joint'] or self.arm.num_joints)
            self._wrist_start = self.arm.get_joint_angles()[self._wrist_joint - 1]
            self._wrist_limit = abs(float(safety['wave']['joint_limit_deg']))
        return bounds

    def _survey_box(self, pose):
        """
        Work out how much of the box is actually usable, before anything moves.

        The box is clipped to the envelope's ranges, but the envelope also keeps a
        cylinder clear around the base column, and a hole in the middle is not something a
        box can be clipped to. So some of the box is normally unusable, and how much is
        worth knowing: a box with very little room left makes the arm look hesitant, which
        the operator should be told is a ``--radius`` away from being fixed rather than
        left to wonder about.

        Only a box with *nothing* usable in it is refused. Anything else is idled in, and
        reported: a wander that has to look a few times before it finds somewhere to go is
        still a wander, and stopping the arm over it would be the wrong way round.

        :raises SafetyError: if there is nowhere in the box to go at all
        """
        sampler = random.Random(0)   # fixed, so the same box always gives the same verdict
        usable = 0
        for _ in range(self.SURVEY_POINTS):
            candidate = [sampler.uniform(low, high) for low, high in self.bounds]
            allowed, _ = self.arm.check_pose_allowed(candidate)
            if allowed and self.arm.check_path_allowed(pose, candidate)[0]:
                usable += 1
        self.usable_fraction = usable / float(self.SURVEY_POINTS)
        if usable == 0:
            raise SafetyError(
                f'there is nowhere in the idle box to wander to: none of '
                f'{self.SURVEY_POINTS} points in it can be reached in a straight line from '
                f'x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f} mm. The box is centred on '
                f'x={self.origin[0]:.1f} y={self.origin[1]:.1f} z={self.origin[2]:.1f} mm, and '
                f'the envelope keeps a keep-out around the base column. Move the arm away '
                f'from the base, or use a smaller radius'
            )
        return usable

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
        ] + ([
            f'  note: only {self.usable_fraction:.0%} of that box can actually be reached '
            f'from here in a straight line, so the arm will pause to look for somewhere to '
            f'go. A smaller radius, or moving it away from the base column, gives it more room'
        ] if self.usable_fraction is not None
             and self.usable_fraction < CRAMPED_BOX_FRACTION else []))

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

    def _arm_fault(self):
        """
        The arm's own complaint, if it has one, or None if it is fit to carry on.

        This is what separates the two kinds of bad news. A rejected move leaves the arm
        exactly as it was and is none of the session's business; a faulted controller
        needs clearing, which is a decision for whoever knows whether a human asked for
        the stop, so it is reported upwards instead of being cleared here.
        """
        try:
            self.arm.assert_ready()
        except SafetyError as fault:
            return fault
        except Exception as fault:                          # an unreadable arm counts too
            return SafetyError(f'the arm could not be checked: {fault}')
        return None

    def _run(self):
        try:
            while not self._stop.is_set():
                try:
                    self._to_waypoint()
                except Exception as error:
                    if self._stop.is_set():
                        # a stop landed mid-move: that is the end of the session, not a
                        # waypoint to retry
                        return
                    fault = self._arm_fault()
                    if fault is not None:
                        self.error = fault
                        return
                    # the arm is fine, so nothing moved and nothing is wrong: a waypoint
                    # it turns out not to be able to walk to is an ordinary event, and
                    # giving up on the idle over one would stop the arm that nobody asked
                    # to have stopped. Try somewhere else, for as long as it takes.
                    self.rejections += 1
                    self.last_rejection = error
                    self._sleep(RETRY_PAUSE_S)
        finally:
            self._stop.set()

    def _pick_waypoint(self):
        """
        A point in the box that the arm can actually be walked to from where it is.

        Three things have to hold, and all three are checked before anything moves:
        the waypoint is inside the envelope, the straight line to it stays inside the
        envelope (the base keep-out is a hole in the middle of the box, so both ends of a
        move can be legal while the middle is not), and the controller agrees it can reach
        the pose at all. The last one is a round trip, so it is asked last, of the few
        candidates that got that far.

        :return: the waypoint, or None if there is nowhere to go from here right now
        """
        pose = self.arm.get_cartesian_pos()
        for _ in range(WAYPOINT_TRIES):
            candidate = [self.rng.uniform(low, high) for low, high in self.bounds]
            if self._walkable(pose, candidate):
                return candidate

        # nothing random worked, so head back to the middle: check_ready proved the arm
        # could idle from the origin, and the walk back is checked like any other
        home = [min(max(self.origin[axis], low), high)
                for axis, (low, high) in enumerate(self.bounds)]
        if self._walkable(pose, home):
            return home
        return None

    def _walkable(self, pose, candidate):
        """
        Whether ``candidate`` is somewhere the arm may be, and may be walked to from ``pose``.

        The free checks go first. ``check_pose_reachable`` is a round trip to the
        controller, so it is asked only about candidates that already passed the rest, and
        is asked about the full pose -- the orientation the arm is holding is half of what
        makes a position reachable at all.
        """
        allowed, _ = self.arm.check_pose_allowed(candidate)
        if not allowed:
            self.refused['envelope'] += 1
            return False
        if not self.arm.check_path_allowed(pose, candidate)[0]:
            self.refused['path'] += 1
            return False
        if not self.ask_controller:
            return True
        reachable, reason = self.arm.check_pose_reachable(
            list(candidate[:3]) + list(pose[3:6]))
        if not reachable:
            self.refused['controller'] += 1
            self.last_rejection = SafetyError(reason)
            return False
        return True

    def _to_waypoint(self):
        """Walk to one waypoint in short validated steps, then maybe pause or twist."""
        target = self._pick_waypoint()
        if target is None:
            # Nowhere to go from this pose, this time round. Waiting is the right answer:
            # the arm is where it should be and still, and the next look may well find
            # somewhere -- the candidates are random, and the pose may yet change.
            self.rejections += 1
            self.last_rejection = SafetyError(
                'no waypoint in the idle box could be walked to from the current pose')
            self._sleep(RETRY_PAUSE_S)
            return
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

        A twist turns the flange rather than translating it, but a tool with any reach at
        all does carry the TCP with it, so the pose is checked afterwards and the wrist is
        put back if it took the gripper somewhere the envelope does not allow. A twist the
        arm refuses outright is skipped: the wander is what matters, and it carries on
        without it rather than ending over a flourish.
        """
        angle = self._wrist_start + self.rng.uniform(-self.wiggle_deg, self.wiggle_deg)
        angle = max(-self._wrist_limit, min(self._wrist_limit, angle))
        settings = self.arm.angle_settings(self.arm.safety['wave']['settings_profile'])
        try:
            self.arm.set_servo_angle(servo_id=self._wrist_joint, angle=angle, **settings)
            self.moves += 1
            allowed, reason = self.arm.check_pose_allowed(self.arm.get_cartesian_pos())
            if not allowed:
                # the twist carried the tool out of the envelope, so undo it: the angle it
                # started from is the one the pose was checked at in check_ready
                self.arm.set_servo_angle(servo_id=self._wrist_joint,
                                         angle=self._wrist_start, **settings)
                self.last_rejection = SafetyError(f'a wrist twist left the envelope ({reason}) '
                                                  f'and was undone')
                self.skipped_wiggles += 1
        except SafetyError as error:
            self.skipped_wiggles += 1
            self.last_rejection = error
        except Exception as error:
            # a controller that refused the twist has to be reported, not swallowed: it
            # may well have latched a fault, which _run's caller is the one to clear
            if self._arm_fault() is not None:
                raise
            self.skipped_wiggles += 1
            self.last_rejection = error
