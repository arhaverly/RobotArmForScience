"""
An offline stand-in for the xArm hardware, for trying things out without the robot.

This is NOT a physics simulation and it knows nothing about the real bench. It answers
the handful of SDK calls ``xArm`` makes -- pose, joint angles, gripper, error state --
so the safety envelope, the plan validator and the VLA action layer can all be exercised
on a laptop. A plan that runs here has been checked against the configured envelope; it
has not been checked against reality.

``xarm_control`` imports the real SDK at module level, so ``install_sdk_stub()`` has to
run before anything imports it. ``make_sim_arm()`` does that for you:

    from robotic_testing.common.robotic_arms.xarm_sim import make_sim_arm
    arm = make_sim_arm('xarm7')
"""
import sys
import types


class SimulatedXArmAPI:
    """Answers the SDK calls that ``xArm`` makes, against an in-memory arm."""

    # a pose comfortably inside both arms' configured envelopes
    START_POSE = {
        'xarm7': [300.0, 300.0, 250.0, 180.0, 0.0, 90.0],
        'xarm6': [500.0, 100.0, 200.0, 180.0, 0.0, 0.0],
    }

    def __init__(self, port=None, **kwargs):
        self.port = port
        self._pose = list(self.START_POSE['xarm6' if str(port).endswith('227') else 'xarm7'])
        self._joints = [0.0] * 7
        self._gripper = 55.0

        # the gripper cannot close past this, which is what lets pick_object see a grasp.
        # set it to 0.0 to simulate closing on empty air.
        self.object_width = 12.0

        self.error_code = 0
        self.warn_code = 0
        self.state = 0
        self.mode = 0
        self.connected = True
        self.axis = 7
        self.commands = []

        class _Arm:
            version_number = (1, 1, 1)

            def release_error_warn_changed_callback(self, *args):
                pass

            def release_state_changed_callback(self, *args):
                pass

            def release_connect_changed_callback(self, *args):
                pass

        self.arm = _Arm()

    # --- lifecycle, all no-ops here -----------------------------------
    def clean_warn(self):
        self.warn_code = 0

    def clean_error(self):
        self.error_code = 0

    def motion_enable(self, enable):
        pass

    def set_mode(self, mode):
        self.mode = mode

    def set_state(self, state):
        self.state = state

    def register_error_warn_changed_callback(self, callback):
        pass

    def register_state_changed_callback(self, callback):
        pass

    def register_count_changed_callback(self, callback):
        pass

    def register_connect_changed_callback(self, callback):
        pass

    def disconnect(self):
        self.connected = False

    # --- reads --------------------------------------------------------
    def get_position(self, **kwargs):
        return 0, list(self._pose)

    def get_servo_angle(self, **kwargs):
        return 0, list(self._joints)

    def get_gripper_position(self):
        return 0, self._gripper

    def get_gripper_version(self):
        return 0, 'simulated'

    # --- writes -------------------------------------------------------
    def set_position(self, *args, **kwargs):
        args = list(args) + [None] * (6 - len(args))
        relative = kwargs.get('relative', False)
        for axis in range(6):
            value = args[axis]
            if value is None:
                continue
            if relative:
                self._pose[axis] += float(value)
            else:
                self._pose[axis] = float(value)
        self.commands.append(('set_position', args[:6], relative))
        return 0

    def set_servo_angle(self, servo_id=None, angle=None, **kwargs):
        if servo_id is None or servo_id == 8:
            angles = list(angle)
            self._joints = [float(value) for value in angles] + [0.0] * (7 - len(angles))
        else:
            self._joints[int(servo_id) - 1] = float(angle)
        self.commands.append(('set_servo_angle', servo_id, angle))
        return 0

    def set_gripper_position(self, pos, **kwargs):
        # closing stops at the width of whatever is between the fingers
        self._gripper = max(float(pos), self.object_width)
        self.commands.append(('set_gripper_position', pos))
        return 0

    # --- xArm6 linear track -------------------------------------------
    def set_linear_track_pos(self, pos, **kwargs):
        self.commands.append(('set_linear_track_pos', pos))
        return 0

    def set_linear_track_enable(self, enable):
        return 0


def install_sdk_stub():
    """
    Put a fake ``xarm`` package in ``sys.modules`` so ``xarm_control`` imports offline.

    Safe to call more than once. Does nothing if the real SDK is already imported, so
    this can never shadow real hardware in a process that is talking to a robot.
    """
    if 'xarm' in sys.modules and not getattr(sys.modules['xarm'], '_is_simulator', False):
        return False

    xarm_module = types.ModuleType('xarm')
    xarm_module._is_simulator = True

    class version:
        __version__ = 'simulated'

    xarm_module.version = version

    wrapper_module = types.ModuleType('xarm.wrapper')
    wrapper_module.XArmAPI = SimulatedXArmAPI
    xarm_module.wrapper = wrapper_module

    sys.modules['xarm'] = xarm_module
    sys.modules['xarm.wrapper'] = wrapper_module
    return True


def make_sim_arm(kind='xarm7'):
    """
    Build a real ``xArm6``/``xArm7`` object backed by the simulator.

    The arm class, the config and every safety check are the genuine ones -- only the
    hardware underneath is fake.
    """
    install_sdk_stub()

    # imported here, after the stub is in place, because xarm_control imports the SDK
    if kind == 'xarm7':
        from robotic_testing.common.robotic_arms.xarm7.xarm7 import xArm7
        return xArm7()
    if kind == 'xarm6':
        from robotic_testing.common.robotic_arms.xarm6.xarm6 import xArm6
        return xArm6()
    raise ValueError(f'unknown arm {kind!r}; expected xarm6 or xarm7')
