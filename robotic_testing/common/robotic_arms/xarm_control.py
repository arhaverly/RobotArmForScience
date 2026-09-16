import time
import traceback
import numpy as np
import importlib
from xarm import version
from xarm.wrapper import XArmAPI


# order of the 6 components of a cartesian TCP pose, as returned by XArmAPI.get_position()
AXIS_NAMES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')

# fallbacks for the free-form-motion settings that are not safety critical.
# the geometric envelope itself (x_range / y_range / z_range) intentionally has NO default:
# if a config does not declare one, free-form motion stays disabled.
DEFAULT_SAFETY = {
    'max_step_mm': 100.0,           # largest single relative translation, per axis and as a vector
    'max_step_deg': 30.0,           # largest single relative rotation per angular axis
    'max_plan_actions': 40,         # largest number of actions accepted in one planned sequence
    'max_wait_s': 60.0,             # largest dwell a plan may request
    'settings_profile': 'slow_1',   # pos_settings_dict key used for free-form motion
    'named_positions': (),          # pos_dict keys a planner is allowed to drive to directly
    'gripper': {
        'object_grip_opening': 0,   # commanded opening when grasping an unknown object
        'object_detect_min': 3.0,   # opening above which we assume something is actually held
    },
    'pick_place': {
        'descend': 60.0,            # default vertical approach distance for pick/place
        'lift': None,               # vertical retreat; None mirrors the descent so pick/place is net-zero in z
        'settings_profile': 'slow_2',
    },
    'wave': {
        'times': 3,
        'amplitude_deg': 25.0,
        'joint': None,              # None -> last joint of the arm
        'joint_limit_deg': 170.0,
        'settings_profile': 'default',
    },
}


class SafetyError(SystemError):
    """Raised when a requested motion is rejected before anything moves."""


class xArm(XArmAPI):
    def __init__(self, config_path, **kwargs):
        # import config python file, config_path example: 'robotic_testing.xarm7.xarm7_config'
        self.config = importlib.import_module(config_path)

        super().__init__(port=self.config.port, **kwargs)

        self.params = {
            'events': {},
            'variables': {},
            'callback_in_thread': True,
            'quit': False
        }

        self.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
        self.clean_warn()
        self.clean_error()
        self.motion_enable(True)
        self.set_mode(0)
        self.set_state(0)

        # register callback functions
        self.register_error_warn_changed_callback(self.error_warn_change_callback)
        self.register_state_changed_callback(self.state_changed_callback)
        self.register_count_changed_callback(self.count_changed_callback)
        self.register_connect_changed_callback(self.connect_changed_callback)

        # sample in flask
        self.sample_in_flask = False

        # a generic (non sample-holder) object is currently grasped, set by pick_object/place_object
        self.holding_object = False

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                       ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    # Register error/warn changed callback
    def error_warn_change_callback(self, data):
        if data and data['error_code'] != 0:
            self.params['quit'] = True
            self.pprint('err={}, quit'.format(data['error_code']))
            self.arm.release_error_warn_changed_callback(self.error_warn_change_callback)

    # Register state changed callback
    def state_changed_callback(self, data):
        if data and data['state'] == 4:
            if self.arm.version_number[0] >= 1 and self.arm.version_number[1] >= 1 and self.arm.version_number[2] > 0:
                self.params['quit'] = True
                self.pprint('state=4, quit')
                self.arm.release_state_changed_callback(self.state_changed_callback)

    # Register counter value changed callback
    def count_changed_callback(self, data):
        if not self.params['quit']:
            self.pprint('counter val: {}'.format(data['count']))

    # Register connect changed callback
    def connect_changed_callback(self, data):
        if data and not data['connected']:
            self.params['quit'] = True
            self.pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
            self.arm.release_connect_changed_callback(self.error_warn_change_callback)

    def check_error(func):
        def error_checker(self, *args, **kwargs):
            if self.error_code == 0 and not self.params['quit']:
                code = func(self, *args, **kwargs)
                if code != 0:
                    self.params['quit'] = True
                    raise SystemError('robotic arm error!')
        return error_checker

    @property
    def has_sample_holder(self):
        threshold = getattr(self.config, 'sample_holder_gripper_threshold', 20)
        return self.gripper_opening < threshold

    def get_gripper_version(self):
        return super().get_gripper_version()

    @check_error
    def set_servo_angle(self, *args, **kwargs):
        return super().set_servo_angle(*args, **{**self.config.ang_settings_dict['default'], **kwargs})

    @check_error
    def set_position(self, *args, **kwargs):
        return super().set_position(*args, **{**self.config.pos_settings_dict['default'], **kwargs})

    def move_to_pos(self, pos, **kwargs):
        return self.set_position(*pos, **kwargs)

    @check_error
    def set_gripper_position(self, *args, **kwargs):
        return super().set_gripper_position(*args, **kwargs)

    def home(self):
        if self.has_sample_holder or self.sample_in_flask:
            raise SystemError('Cannot go home, unload sample first!')
        else:
            self.stretch()
            self.set_servo_angle(angle=self.config.ang_dict['home'])

    def stretch(self):
        if self.has_sample_holder or self.sample_in_flask:
            raise SystemError('Cannot stretch, unload sample first!')
        else:
            self.set_servo_angle(angle=self.config.ang_dict['stretch'])

    def close_gripper(self):
        self.set_gripper_position(0, wait=True, speed=5000, auto_enable=True)

    def open_gripper(self):
        self.set_gripper_position(self.config.gripper_open_dist, wait=True, speed=5000, auto_enable=True)

    def move_to_hover_pos(self, pos, hover_offset, **kwargs):
        # Raise vertically first to avoid lateral motion while rising.
        self.set_position(0, 0, hover_offset, 0, 0, 0, relative=True)
        pos_hover = pos.copy()
        pos_hover[2] += hover_offset
        self.move_to_pos(pos_hover, **kwargs)

    def pick_up_sample(self, sample_rack_index):
        raise NotImplementedError('sample pick up needs to be specified in the child class')

    def put_sample_back(self, sample_rack_index):
        raise NotImplementedError('sample put back needs to be specified in the child class')

    def upright_sample(self, reverse=False):
        if reverse:
            self.move_to_pos(
                self.config.pos_dict['vertical_rotation_reverse'],
                relative=True,
                # radius=1,
                **self.config.pos_settings_dict['default']
            )
        else:
            self.move_to_pos(
                self.config.pos_dict['vertical_rotation'],
                relative=True,
                **self.config.pos_settings_dict['default']
            )

    def move_to_mid_station(self):
        self.move_to_pos(self.config.pos_dict['mid_station'])

    def move_to_flask(self):
        self.move_to_hover_pos(self.config.pos_dict['flask_contact_immersed'], self.config.hover_offset_dict['flask'])

    def sink_in_flask(self, reverse=False, immerse_option='flask_contact_immersed'):
        if reverse:
            self.move_to_hover_pos(
                self.config.pos_dict['flask_contact_immersed'],
                self.config.hover_offset_dict['flask'],
                **self.config.pos_settings_dict['slow_3']
            )
            self.sample_in_flask = False
        else:
            self.move_to_pos(self.config.pos_dict[immerse_option], **self.config.pos_settings_dict['slow_3'])
            self.sample_in_flask = True

    def sink_in_flask_controlled_speed(self, reverse=False, immerse_option='flask_contact_immersed', speed=0.5):
        if reverse:
            self.move_to_hover_pos(
                self.config.pos_dict['flask_contact_immersed'],
                self.config.hover_offset_dict['flask'],
                **self.config.pos_settings_dict[f'megnan_{speed}']
            )
            self.sample_in_flask = False
        else:
            self.move_to_pos(self.config.pos_dict[immerse_option], **self.config.pos_settings_dict[f'megnan_{speed}'])
            self.sample_in_flask = True

    def rinsing(self):
        raise NotImplementedError('function needs to be specified in the child class')

    def load_sample(self, sample_rack_index, **kwargs):
        raise NotImplementedError('function needs to be specified in the child class')

    def unload_sample(self, sample_rack_index):
        raise NotImplementedError('function needs to be specified in the child class')

    # ------------------------------------------------------------------
    # free-form motion: state inspection
    # ------------------------------------------------------------------

    @property
    def safety(self):
        """
        The merged safety configuration of this arm.

        Values come from ``safety_dict`` in the arm config file, on top of DEFAULT_SAFETY.
        An arm whose config does not declare a ``safety_dict`` cannot be driven with the
        free-form helpers at all: there is no verified envelope to keep it inside.
        """
        safety_dict = getattr(self.config, 'safety_dict', None)
        if safety_dict is None:
            raise SafetyError(
                f'{self.config.__name__} declares no safety_dict, so free-form motion is disabled. '
                f'Add x_range / y_range / z_range (arm base coordinates, mm) after physically '
                f'verifying the reachable volume is clear.'
            )
        merged = {**DEFAULT_SAFETY, **safety_dict}
        # nested dicts are merged one level deep so a config can override a single key
        for key in ('gripper', 'pick_place', 'wave'):
            merged[key] = {**DEFAULT_SAFETY[key], **safety_dict.get(key, {})}
        return merged

    def _envelope(self):
        """Return the safety config, guaranteeing the geometric envelope is fully declared."""
        safety = self.safety
        missing = [key for key in ('x_range', 'y_range', 'z_range') if safety.get(key) is None]
        if missing:
            raise SafetyError(
                f'{self.config.__name__} safety_dict is missing {missing}; free-form motion is disabled'
            )
        return safety

    @property
    def num_joints(self):
        """Number of joints of this arm (6 for the xArm6, 7 for the xArm7)."""
        axis = getattr(self, 'axis', None)
        return int(axis) if axis else len(self.get_joint_angles())

    def get_cartesian_pos(self):
        """Current TCP pose [x, y, z, roll, pitch, yaw] in arm base coordinates (mm / degrees)."""
        code, pos = self.get_position()
        if code != 0 or pos is None:
            raise SystemError(f'failed to read the TCP position, code={code}')
        return [float(value) for value in pos]

    def get_joint_angles(self):
        """Current joint angles in degrees, joint 1 first."""
        code, angles = self.get_servo_angle()
        if code != 0 or angles is None:
            raise SystemError(f'failed to read the joint angles, code={code}')
        return [float(angle) for angle in angles]

    @property
    def gripper_opening(self):
        """Current gripper opening in mm."""
        code, opening = super().get_gripper_position()
        if code != 0 or opening is None:
            raise SystemError(f'failed to read the gripper position, code={code}')
        return float(opening)

    def assert_ready(self):
        """
        Refuse to start a motion unless the arm is in a known-good state.

        This matters because the ``check_error`` decorator silently skips a motion command
        when the arm is already faulted, which otherwise looks like a move that succeeded.
        """
        if not self.connected:
            raise SafetyError('arm is not connected')
        if self.params['quit']:
            raise SafetyError(
                'arm is latched into the quit state after an earlier fault; '
                'clear the cause and call reset_safety_state()'
            )
        if self.error_code != 0:
            raise SafetyError(f'arm reports error_code={self.error_code}; call reset_safety_state() once it is safe')
        if self.state == 4:
            raise SafetyError('arm is in the stop state (state=4); call reset_safety_state() once it is safe')

    def reset_safety_state(self):
        """Clear errors/warnings and re-enable motion. Only call this once the cause is understood."""
        self.clean_warn()
        self.clean_error()
        self.motion_enable(True)
        self.set_mode(0)
        self.set_state(0)
        self.params['quit'] = False
        return self.describe_state()

    def emergency_stop(self):
        """Stop the arm immediately. Recover with reset_safety_state()."""
        self.set_state(4)
        self.params['quit'] = True

    def describe_state(self):
        """A snapshot of everything a planner needs to reason about the next move."""
        pose = self.get_cartesian_pos()
        state = {
            'pose': {name: round(value, 2) for name, value in zip(AXIS_NAMES, pose)},
            'joint_angles_deg': [round(angle, 2) for angle in self.get_joint_angles()],
            'holding_object': self.holding_object,
            'sample_in_flask': self.sample_in_flask,
            'error_code': self.error_code,
            'state': self.state,
        }
        # reporting state must not fail just because the gripper cannot be read
        try:
            state['gripper_opening_mm'] = round(self.gripper_opening, 2)
            state['has_sample_holder'] = self.has_sample_holder
        except SystemError as error:
            state['gripper_opening_mm'] = None
            state['has_sample_holder'] = None
            state['gripper_error'] = str(error)
        try:
            safety = self._envelope()
        except SafetyError as error:
            state['envelope'] = str(error)
            return state
        state['envelope'] = {
            'x_range': list(safety['x_range']),
            'y_range': list(safety['y_range']),
            'z_range': list(safety['z_range']),
            'max_step_mm': safety['max_step_mm'],
            'max_step_deg': safety['max_step_deg'],
        }
        allowed, reason = self.check_pose_allowed(pose)
        state['inside_envelope'] = allowed
        if not allowed:
            state['envelope_violation'] = reason
        return state

    def describe_envelope(self):
        """Human/LLM readable description of the limits that free-form motion is held to."""
        safety = self._envelope()
        lines = [
            'Allowed TCP volume (arm base coordinates, mm):',
            '  x: {} .. {}'.format(*safety['x_range']),
            '  y: {} .. {}'.format(*safety['y_range']),
            '  z: {} .. {}'.format(*safety['z_range']),
            f'Largest single translation: {safety["max_step_mm"]} mm (per axis and as a vector)',
            f'Largest single rotation: {safety["max_step_deg"]} deg per axis',
            f'Largest number of actions in one plan: {safety["max_plan_actions"]}',
        ]
        if safety.get('max_reach_mm'):
            lines.append(f'Largest horizontal distance from the base: {safety["max_reach_mm"]} mm')
        if safety.get('min_reach_mm'):
            lines.append(f'Smallest horizontal distance from the base: {safety["min_reach_mm"]} mm')
        if safety.get('named_positions'):
            lines.append('Named positions available: {}'.format(', '.join(safety['named_positions'])))
        return '\n'.join(lines)

    # ------------------------------------------------------------------
    # free-form motion: validation
    # ------------------------------------------------------------------

    def check_step_size(self, delta):
        """
        Check a relative offset against the single-step limits, independently of where the
        arm currently is. Returns the offset padded to 6 components.

        Capping the step size is what keeps a planner from requesting one huge sweep across
        the bench: a long move has to be expressed as several short ones, each of which is
        validated against the envelope on its own.
        """
        delta = list(delta)
        if len(delta) > 6:
            raise SafetyError(f'a relative move takes at most 6 components, got {len(delta)}')
        delta = [float(value) for value in delta] + [0.0] * (6 - len(delta))
        safety = self._envelope()
        max_step = float(safety['max_step_mm'])
        max_turn = float(safety['max_step_deg'])

        for axis, value in enumerate(delta[:3]):
            if abs(value) > max_step:
                raise SafetyError(
                    f'requested d{AXIS_NAMES[axis]}={value:.1f} mm exceeds the '
                    f'{max_step} mm single-step limit; break the move into smaller steps'
                )
        distance = float(np.linalg.norm(delta[:3]))
        if distance > max_step:
            raise SafetyError(
                f'requested step of {distance:.1f} mm exceeds the {max_step} mm single-step limit; '
                f'break the move into smaller steps'
            )
        for axis, value in enumerate(delta[3:6], start=3):
            if abs(value) > max_turn:
                raise SafetyError(
                    f'requested d{AXIS_NAMES[axis]}={value:.1f} deg exceeds the {max_turn} deg single-step limit'
                )
        return delta

    def check_pose_allowed(self, pose):
        """
        Check an absolute TCP pose against the configured envelope.

        :param pose: [x, y, z, ...]; entries beyond z are ignored, None entries are skipped
        :return: (allowed, reason) -- reason is '' when allowed
        """
        safety = self._envelope()

        # every violation is reported, not just the first: being told one bad axis at a
        # time turns diagnosing a badly calibrated envelope into a guessing game
        reasons = []
        for axis, key in ((0, 'x_range'), (1, 'y_range'), (2, 'z_range')):
            value = pose[axis] if axis < len(pose) else None
            if value is None:
                continue
            low, high = safety[key]
            if not low <= value <= high:
                reasons.append(f'{AXIS_NAMES[axis]}={value:.1f} mm is outside the allowed range [{low}, {high}]')

        x, y = pose[0], pose[1]
        if x is not None and y is not None:
            reach = float(np.hypot(x, y))
            max_reach = safety.get('max_reach_mm')
            min_reach = safety.get('min_reach_mm')
            if max_reach and reach > max_reach:
                reasons.append(f'horizontal reach {reach:.1f} mm exceeds the limit of {max_reach} mm')
            if min_reach and reach < min_reach:
                reasons.append(
                    f'horizontal reach {reach:.1f} mm is inside the {min_reach} mm keep-out around the base'
                )
        return (not reasons), '; '.join(reasons)

    def assert_pose_allowed(self, pose):
        allowed, reason = self.check_pose_allowed(pose)
        if not allowed:
            raise SafetyError(reason)

    def plan_relative_move(self, delta, from_pose=None):
        """
        Validate a relative move without executing anything.

        Both the step size and the resulting pose are checked. Because ``set_position``
        moves the TCP along a straight line and the envelope is an axis-aligned box, a
        move whose start and end are both inside the box keeps the whole path inside it.

        :param delta: [dx, dy, dz] or [dx, dy, dz, droll, dpitch, dyaw] in mm / degrees
        :param from_pose: pose to plan from; defaults to the current pose
        :return: the predicted target pose
        :raises SafetyError: if the move would break a limit
        """
        delta = self.check_step_size(delta)
        start = list(from_pose) if from_pose is not None else self.get_cartesian_pos()
        allowed, reason = self.check_pose_allowed(start)
        if not allowed:
            raise SafetyError(f'refusing to move: the arm is already outside the envelope ({reason})')

        target = [start[axis] + delta[axis] for axis in range(6)]
        allowed, reason = self.check_pose_allowed(target)
        if not allowed:
            raise SafetyError(f'refusing to move: the target pose would be unsafe ({reason})')
        return target

    def plan_absolute_move(self, pose, from_pose=None):
        """
        Validate an absolute move. ``None`` components keep their current value.

        :return: the predicted target pose
        """
        pose = list(pose) + [None] * (6 - len(pose))
        start = list(from_pose) if from_pose is not None else self.get_cartesian_pos()
        target = [start[axis] if pose[axis] is None else float(pose[axis]) for axis in range(6)]
        delta = [target[axis] - start[axis] for axis in range(6)]
        self.plan_relative_move(delta, from_pose=start)
        return target

    # ------------------------------------------------------------------
    # free-form motion: execution
    # ------------------------------------------------------------------

    def motion_settings(self, profile=None):
        """pos_settings kwargs for a named profile, or the free-form default."""
        profile = profile or self.safety['settings_profile']
        if isinstance(profile, dict):
            return dict(profile)
        try:
            return dict(self.config.pos_settings_dict[profile])
        except KeyError:
            raise SafetyError(
                f'unknown position settings profile {profile!r}; '
                f'available: {sorted(self.config.pos_settings_dict)}'
            )

    def angle_settings(self, profile=None):
        """ang_settings kwargs for a named profile."""
        profile = profile or 'default'
        if isinstance(profile, dict):
            return dict(profile)
        try:
            return dict(self.config.ang_settings_dict[profile])
        except KeyError:
            raise SafetyError(
                f'unknown angle settings profile {profile!r}; '
                f'available: {sorted(self.config.ang_settings_dict)}'
            )

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0,
                      settings_profile=None, dry_run=False):
        """
        Move the TCP by an arbitrary relative offset, after checking it is safe.

        Translations are in the arm base frame, in mm; rotations are in degrees.
        The move is rejected before anything happens if the arm is faulted, if the step
        is larger than the configured single-step limit, or if start or target lie
        outside the configured envelope.

        :param settings_profile: pos_settings_dict key (or an explicit kwargs dict);
            defaults to the deliberately slow free-form profile from safety_dict
        :param dry_run: only validate and return the predicted pose
        :return: the pose reached (or predicted, when dry_run)
        """
        self.assert_ready()
        delta = [dx, dy, dz, droll, dpitch, dyaw]
        target = self.plan_relative_move(delta)
        if dry_run:
            return target
        self.set_position(*delta, relative=True, **self.motion_settings(settings_profile))
        return self.get_cartesian_pos()

    def move_to_pose(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None,
                     settings_profile=None, dry_run=False):
        """
        Move the TCP to an absolute pose inside the envelope. ``None`` keeps the current value.

        The single-step limit applies here too, so this is for nudging to a nearby pose,
        not for crossing the workspace; string several moves together, or use
        ``move_to_named_position`` for the poses the config has verified as safe to jump to.

        Note for the xArm6: this drives the arm only. The linear track is not involved,
        so the coordinates here are the arm's own, as reported by get_cartesian_pos().
        """
        self.assert_ready()
        target = self.plan_absolute_move([x, y, z, roll, pitch, yaw])
        if dry_run:
            return target
        self.set_position(*target, **self.motion_settings(settings_profile))
        return self.get_cartesian_pos()

    def move_relative_path(self, deltas, settings_profile=None, dry_run=False):
        """
        Run a sequence of relative moves, validating the whole path before the first one.

        :param deltas: iterable of [dx, dy, dz] (or 6-component) offsets
        :return: list of poses reached (or predicted, when dry_run)
        """
        deltas = [list(delta) for delta in deltas]
        safety = self._envelope()
        if len(deltas) > safety['max_plan_actions']:
            raise SafetyError(
                f'{len(deltas)} steps exceeds the limit of {safety["max_plan_actions"]} steps per plan'
            )

        # validate the full path first, so a bad step never leaves the arm half way through
        pose = self.get_cartesian_pos()
        predicted = []
        for index, delta in enumerate(deltas):
            try:
                pose = self.plan_relative_move(delta, from_pose=pose)
            except SafetyError as error:
                raise SafetyError(f'step {index + 1} of {len(deltas)} rejected: {error}')
            predicted.append(pose)
        if dry_run:
            return predicted

        reached = []
        for delta in deltas:
            reached.append(self.move_relative(*delta, settings_profile=settings_profile))
        return reached

    def move_up(self, distance, **kwargs):
        """Raise the TCP vertically. Handy building block for approach/retreat moves."""
        return self.move_relative(dz=abs(distance), **kwargs)

    def move_down(self, distance, **kwargs):
        """Lower the TCP vertically."""
        return self.move_relative(dz=-abs(distance), **kwargs)

    # ------------------------------------------------------------------
    # free-form manipulation
    # ------------------------------------------------------------------

    def set_gripper_opening(self, opening, wait=True, speed=5000):
        """Drive the gripper to an opening in mm, clamped to the mechanical range."""
        limit = float(self.config.gripper_open_dist)
        opening = float(min(max(opening, 0.0), limit))
        self.set_gripper_position(opening, wait=wait, speed=speed, auto_enable=True)
        return opening

    def pick_object(self, descend=None, lift=None, grip_opening=None, settings_profile=None, verify=True):
        """
        Pick up whatever is directly below the gripper.

        The arm must already be positioned above the object -- use move_relative to get
        there. The sequence is: open, descend, close, retreat vertically.

        :param descend: how far to come down onto the object, mm
        :param lift: how far to retreat afterwards, mm (defaults to ``descend``)
        :param grip_opening: opening commanded when closing onto the object, mm
        :param verify: warn if the gripper closed all the way, which means it caught nothing
        :return: dict describing the outcome
        """
        self.assert_ready()
        if self.holding_object:
            raise SafetyError('already holding an object; place it down before picking up another one')
        if self.sample_in_flask:
            raise SafetyError('a sample is still in the flask; finish that workflow first')

        settings = self.safety['pick_place']
        descend = float(settings['descend'] if descend is None else descend)
        lift = float(lift if lift is not None else (settings['lift'] or descend))
        profile = settings_profile or settings['settings_profile']
        grip_opening = self.safety['gripper']['object_grip_opening'] if grip_opening is None else grip_opening

        # validate both legs of the motion before touching anything
        down_pose = self.plan_relative_move([0, 0, -descend])
        self.plan_relative_move([0, 0, lift], from_pose=down_pose)

        self.open_gripper()
        self.move_relative(dz=-descend, settings_profile=profile)
        self.set_gripper_opening(grip_opening)

        opening = self.gripper_opening
        grasped = opening > self.safety['gripper']['object_detect_min']
        if verify and not grasped:
            self.pprint(
                f'pick_object: gripper closed to {opening:.1f} mm, so nothing appears to be held'
            )

        self.move_relative(dz=lift, settings_profile=profile)
        self.holding_object = grasped
        return {'grasped': grasped, 'gripper_opening_mm': round(opening, 2), 'pose': self.get_cartesian_pos()}

    def place_object(self, descend=None, lift=None, settings_profile=None, force=False):
        """
        Put the held object down below the gripper and retreat.

        The arm must already be positioned above the drop point.

        :param descend: how far to lower the object before releasing, mm
        :param lift: how far to retreat after releasing, mm (defaults to ``descend``)
        :param force: release even when no grasp was recorded
        """
        self.assert_ready()
        if not force and not self.holding_object and self.gripper_opening >= float(self.config.gripper_open_dist) - 1:
            raise SafetyError(
                'the gripper is already fully open and nothing is recorded as held, so there is '
                'nothing to place; pass force=True to run the motion anyway'
            )

        settings = self.safety['pick_place']
        descend = float(settings['descend'] if descend is None else descend)
        lift = float(lift if lift is not None else (settings['lift'] or descend))
        profile = settings_profile or settings['settings_profile']

        down_pose = self.plan_relative_move([0, 0, -descend])
        self.plan_relative_move([0, 0, lift], from_pose=down_pose)

        self.move_relative(dz=-descend, settings_profile=profile)
        self.open_gripper()
        self.holding_object = False
        self.move_relative(dz=lift, settings_profile=profile)
        return {'pose': self.get_cartesian_pos()}

    def wave(self, times=None, amplitude_deg=None, joint=None, settings_profile=None):
        """
        Wave at the user by rocking one wrist joint back and forth.

        Rotating the last joint keeps the TCP essentially in place, so this is safe to do
        from wherever the arm happens to be standing -- no envelope excursion involved.
        It is refused while anything is held, so a wave can never fling a sample around.
        """
        self.assert_ready()
        if self.sample_in_flask:
            raise SafetyError('refusing to wave while a sample is in the flask; unload it first')
        if self.holding_object:
            raise SafetyError('refusing to wave while holding an object; place it down first')
        if self.has_sample_holder:
            raise SafetyError(
                f'refusing to wave with the gripper closed to {self.gripper_opening:.1f} mm, since it may '
                f'be holding something; open the gripper first'
            )

        settings = self.safety['wave']
        times = int(settings['times'] if times is None else times)
        amplitude = abs(float(settings['amplitude_deg'] if amplitude_deg is None else amplitude_deg))
        joint = int(joint or settings['joint'] or self.num_joints)
        limit = abs(float(settings['joint_limit_deg']))
        kwargs = self.angle_settings(settings_profile or settings['settings_profile'])

        if not 1 <= joint <= self.num_joints:
            raise SafetyError(f'joint {joint} does not exist on this {self.num_joints}-axis arm')
        if times < 1:
            raise SafetyError('wave needs at least one cycle')

        start = self.get_joint_angles()[joint - 1]
        high = min(start + amplitude, limit)
        low = max(start - amplitude, -limit)
        if high - low < 1.0:
            raise SafetyError(
                f'joint {joint} sits at {start:.1f} deg, too close to the +/-{limit} deg wave limit to swing'
            )

        for _ in range(times):
            self.set_servo_angle(servo_id=joint, angle=high, **kwargs)
            self.set_servo_angle(servo_id=joint, angle=low, **kwargs)
        self.set_servo_angle(servo_id=joint, angle=start, **kwargs)
        return {'joint': joint, 'cycles': times, 'from_deg': round(low, 1), 'to_deg': round(high, 1)}

    def move_to_named_position(self, name, settings_profile=None):
        """
        Drive to one of the positions the config explicitly whitelists for free-form use.

        Only keys listed in ``safety_dict['named_positions']`` are reachable this way, so a
        planner cannot stumble into a station-specific pose such as an immersed flask depth.
        """
        safety = self._envelope()
        allowed = tuple(safety['named_positions'])
        if name not in allowed:
            raise SafetyError(f'{name!r} is not a whitelisted named position; available: {list(allowed)}')
        pose = list(self.config.pos_dict[name])
        self.assert_ready()
        self.assert_pose_allowed(pose)
        self.move_to_pos(pose, **self.motion_settings(settings_profile))
        return self.get_cartesian_pos()

    def safe_home(self):
        """
        Return to the configured home pose, refusing while anything is held.

        Unlike ``home()`` this reports exactly what is in the way, which is what a planner
        needs in order to fix the situation itself.
        """
        self.assert_ready()
        if self.holding_object:
            raise SafetyError('still holding an object; place it down before going home')
        if self.sample_in_flask:
            raise SafetyError('a sample is still in the flask; unload it before going home')
        if self.has_sample_holder:
            raise SafetyError('the gripper is closed on something; open it before going home')
        self.home()
        return self.get_cartesian_pos()
