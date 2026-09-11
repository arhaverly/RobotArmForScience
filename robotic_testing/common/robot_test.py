import logging
import os
from abc import ABC, abstractmethod
from datetime import datetime
from functools import wraps
from time import sleep
from typing import List, Callable
import pandas as pd
from software_control.software_control import EC_Lab, CameraControl
from db_control.database import Database
from robotic_testing.common.robotic_arms.xarm_control import xArm, SafetyError
from utils.utils import log_and_print, config_loader, get_dir, email, get_log_path, get_logger
import pyautogui


class RobotTest(ABC):
    def __init__(
            self,
            exp_name: str,
            ec_lab: EC_Lab,
            camera: CameraControl,
            arm: xArm,
            db: Database = None,
            to_log=True,
            prompt_window_bypass_list: List[Callable[[], None]] = None,
            *args,
            **kwargs
    ):
        self.exp_name = exp_name
        self.configs = config_loader(exp_name, config_type='robot_test_configs')
        self.to_log = to_log
        self.ec_lab = ec_lab
        self.camera = camera
        self.db = db
        self.arm = arm
        self.prompt_window_bypass_list = prompt_window_bypass_list

        # logger settings
        self.logger = get_logger(exp_name=exp_name, module_name=self.__class__.__name__) if self.to_log else None

    def log_errors(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            try:
                return func(self, *args, **kwargs)
            except Exception as e:
                if self.logger is not None:
                    self.logger.exception(e)
                raise

        return wrapper

    @staticmethod
    def get_cur_time(time_format='log'):
        assert time_format in ['log', 'file_name', 'sql']
        cur_time = datetime.now()
        if time_format == 'log':
            return cur_time.strftime('%Y-%m-%d %H:%M:%S')
        elif time_format == 'file_name':
            return cur_time.strftime('%Y-%m-%d_%H-%M-%S')
        else:
            return cur_time.replace(microsecond=0)

    def get_ec_lab_file_name(self, test_name, technique_id, channel_id):
        technique = self.configs['protocol'][technique_id]
        if int(technique_id) < 10:
            technique_id = f'0{technique_id}'
        return f'{self.configs["data_dir"]}/{test_name}_{technique_id}_{technique}_C0{channel_id}'

    def analyze_test_results(self, sample_id, test_name, channel_id, **kwargs):
        data_dict = {'sample_id': [sample_id]}
        analysis_config = self.configs['data_analysis']
        for metric_name, metric_info in analysis_config['metrics'].items():
            technique_id = metric_info['technique_id']
            technique = self.configs['protocol'][technique_id]
            file_name = self.get_ec_lab_file_name(test_name, technique_id, channel_id)
            sample_area = self.configs['sample_area']
            analyzer = analysis_config['analyzer']['analyzer_class'](
                file_name,
                technique=technique,
                sample_area=sample_area,
                **{'kwargs': {}, **analysis_config['analyzer']}.get('kwargs')
            )
            analyze_result = metric_info['analyze_method'](analyzer, **{'kwargs': {}, **metric_info}.get('kwargs'))
            data_dict[metric_name] = [analyze_result]
        data_df = pd.DataFrame(data_dict)
        return data_df

    def run_initialization(
            self,
            sequence,
            sample_id_list,
            sample_rack_id_starting,
            benchmark
    ):
        """
        :param sequence: True if the tested sample index is in a continuous sequence, False if not in sequence
        :param sample_id_list: If sequence is True, input sample starting and ending index, e.g. [3, 10] for sample #3
        to #10. If sequence is False, input a list in the exact order of the sample placed on the sample rack,
        e.g. [3, 5, 2, 10] for sample #3, #5, #2, #10 respectively.
        :param sample_rack_id_starting: the starting id on sample rack, starting with 0
        :param benchmark: the benchmark sample id, if provided, the robot will test the benchmark sample first, format in {sample_rack_id: sample_id}
        """
        sample_id_list = [*range(sample_id_list[0], sample_id_list[1] + 1)] if sequence else sample_id_list
        task_len = len(sample_id_list)
        sample_rack_id_list = [*range(sample_rack_id_starting, sample_rack_id_starting + task_len)]
        sample_rack_id_to_sample_id_dict = dict(zip(sample_rack_id_list, sample_id_list))
        if benchmark:
            # check the sample_rack_id in benchmark does not conflict with the sample_rack_id in the task
            assert not set(benchmark.keys()).intersection(
                set(sample_rack_id_to_sample_id_dict.keys())), 'sample_rack_id in benchmark conflicts with the sample_rack_id in the task'
            # add benchmark to the task
            sample_rack_id_to_sample_id_dict = {**benchmark, **sample_rack_id_to_sample_id_dict}
        msg = f'started a batch of test, with sample_rack_id : sample_id = ' \
              f'{sample_rack_id_to_sample_id_dict}'
        log_and_print(self.to_log, self.logger, 'info', msg)
        return sample_rack_id_to_sample_id_dict

    def get_test_name(self, sample_id):
        return f'{self.exp_name}_sample#{sample_id}_{self.get_cur_time("file_name")}'

    @abstractmethod
    def test_main_workflow(self, *args, **kwargs):
        pass

    @abstractmethod
    def robot_warming_up(self):
        pass

    @abstractmethod
    def robot_returning_home(self):
        pass

    @abstractmethod
    def check_test_finished(self):
        pass

    def data_analyze(self, sample_id, test_name, channel_id):
        assert isinstance(self.db, Database), 'database must be initialized to use data analysis'
        analyze_result = self.analyze_test_results(sample_id, test_name, channel_id)
        self.db.add_data_to_server(self.db.configs['tables_dict']['active_learning.performance'], analyze_result)

    @staticmethod
    def mouse_moved(time_frame=10):
        x, y = pyautogui.position()
        sleep(time_frame)
        # check if human is operating PC, any cursor movement during 10s
        if (x, y) == pyautogui.position():
            return False
        else:
            return True


    def bypass_prompt_window(self):
        for bypass_func in self.prompt_window_bypass_list:
            bypass_func()

    def send_email(self, operator_name, sample_rack_id_to_sample_id_dict):
        email_address = self.configs['email'][operator_name]
        task_history = "\n".join(f"{k} : {v}" for k, v in sample_rack_id_to_sample_id_dict.items())
        content = f"{self.exp_name} finished with samples tested as follows:\n\n" \
                  f"{task_history}\n\n" \
                  f"Your research assistant,\n" \
                  f"Crest"
        email(email_address, f'[success] {self.exp_name} robotic testing finished', content)

    @log_errors
    def run(
            self,
            sequence,
            sample_id_list,
            sample_rack_id_starting,
            immerse_option='flask_contact_immersed',
            data_analysis=False,
            benchmark=None,
            operator_name=None,
    ):

        sample_rack_id_to_sample_id_dict = self.run_initialization(
            sequence, sample_id_list, sample_rack_id_starting, benchmark
        )

        self.robot_warming_up()

        for sample_rack_id, sample_id in sample_rack_id_to_sample_id_dict.items():
            # get test name
            test_name = self.get_test_name(sample_id)

            # start testing
            self.test_main_workflow(
                sample_rack_id=sample_rack_id,
                test_name=test_name,
                immerse_option=immerse_option,
            )

            # data analysis
            if data_analysis:
                channel_id = self.ec_lab.channel_id
                # give some time for the software to export csv file
                while True:
                    try:
                        sleep(2)
                        self.data_analyze(sample_id, test_name, channel_id)
                        break
                    except FileNotFoundError:
                        self.logger.warning(f'test raw file not found, retrying...')
                        continue

        self.robot_returning_home()

        # send the notification email if operator name is provided
        if operator_name:
            self.send_email(operator_name, sample_rack_id_to_sample_id_dict)


class RobotTest_AlkalinePlatform(RobotTest):

    def test_main_workflow(self, sample_rack_id, test_name, immerse_option, *args, **kwargs):
        # start recording
        self.camera.start()

        # move to mid-station
        self.arm.move_to_mid_station()

        # load sample
        self.logger.info(f'loading the sample from rack_id #{sample_rack_id}...')
        self.arm.load_sample(sample_rack_index=sample_rack_id, immerse_option=immerse_option)
        self.logger.info(f'sample successfully loaded into the cell')

        # stop recording
        self.camera.stop()
        sleep(3)

        # open software and start testing
        self.ec_lab.start(test_name)

        # wait and bypass warning window
        sleep(10)

        # check test finished
        self.check_test_finished()
        self.logger.info(f'test {test_name} finished')

        # start recording
        self.camera.start()

        # unload sample
        self.logger.info(f'unloading the sample to rack_id #{sample_rack_id}...')
        self.arm.unload_sample(sample_rack_index=sample_rack_id)
        self.logger.info(f'sample successfully unloaded from the cell')

        # stop recording
        sleep(3)
        self.camera.stop()

    def robot_warming_up(self):
        self.arm.home()

    def robot_returning_home(self):
        self.arm.home()

    def check_test_finished(self):
        while self.ec_lab.running:
            # check test status if no human movement in 10 seconds
            if not self.mouse_moved():
                # bypass teamviewer window if exist
                self.bypass_prompt_window()
                # open software
                self.ec_lab.open_software()
                # check test status
                if self.ec_lab.test_is_complete():
                    self.ec_lab.running = False
            else:
                self.logger.info(f'human operating, fail to detect...')


class RobotTest_AcidicPlatform(RobotTest):

    def test_main_workflow(self, sample_rack_id, test_name, *args, **kwargs):
        # start recording
        self.camera.start()

        # load sample
        self.logger.info(f'loading the sample from rack_id #{sample_rack_id}...')
        self.arm.load_sample(sample_rack_index=sample_rack_id)
        self.logger.info(f'sample successfully loaded into the cell')

        # stop recording
        self.camera.stop()
        sleep(3)

        # open software and start testing
        self.ec_lab.start(test_name)
        sleep(10)

        # check test finished
        self.check_test_finished()
        self.logger.info(f'test {test_name} finished')

        # start recording
        self.camera.start()

        # unload sample
        self.logger.info(f'unloading the sample to rack_id #{sample_rack_id}...')
        self.arm.unload_sample(sample_rack_index=sample_rack_id)
        self.logger.info(f'sample successfully unloaded from the cell')

        # stop recording
        sleep(3)
        self.camera.stop()

    def robot_warming_up(self):
        self.arm.home()
        self.arm.warm_up()

    def robot_returning_home(self):
        self.arm.warm_up()
        self.arm.home()

    def check_test_finished(self):
        while self.ec_lab.running:
            # check test status if no human movement in 10 seconds
            if not self.mouse_moved():
                # bypass prompt window if exist
                self.bypass_prompt_window()
                # open software
                self.ec_lab.open_software()
                # open channel
                self.ec_lab.open_channel()
                # check test status
                if self.ec_lab.test_is_complete():
                    self.ec_lab.running = False
            else:
                self.logger.info(f'human operating, fail to detect...')


    

class VLARobot:
    """
    The action surface a VLA / LLM planner is allowed to touch.

    Everything a planner can ask the arm to do goes through ``ACTION_SPECS``: a fixed,
    documented vocabulary with typed and bounded parameters. Three things follow from
    keeping the vocabulary here rather than in the notebook:

    * the planner prompt is generated from the same registry that executes the actions,
      so the two cannot drift apart;
    * a plan is validated in full -- parameter types, step sizes and the simulated TCP
      path against the arm's safety envelope -- before the first action runs;
    * anything not in the registry simply cannot be called, so a hallucinated action
      name is a rejected plan rather than an unexpected motion.

    The envelope itself lives in the arm config (``safety_dict``) and is enforced inside
    ``xArm``, so these checks are a fast-failing front door, not the only line of defence.
    """

    ACTION_SPECS = {
        'get_state': {
            'summary': 'Report the current pose, gripper opening and limits. Moves nothing.',
            'params': {},
            'run': 'run_get_state',
        },
        'move_relative': {
            'summary': 'Move the gripper by a relative offset in mm. '
                       '+x/-x, +y/-y and +z/-z are the arm base axes; +z is up.',
            'params': {
                'dx': {'type': float, 'default': 0.0, 'doc': 'offset along x in mm'},
                'dy': {'type': float, 'default': 0.0, 'doc': 'offset along y in mm'},
                'dz': {'type': float, 'default': 0.0, 'doc': 'offset along z in mm, positive is up'},
            },
            'run': 'run_move_relative',
        },
        'rotate_relative': {
            'summary': 'Rotate the gripper by a relative offset in degrees.',
            'params': {
                'droll': {'type': float, 'default': 0.0, 'doc': 'roll offset in degrees'},
                'dpitch': {'type': float, 'default': 0.0, 'doc': 'pitch offset in degrees'},
                'dyaw': {'type': float, 'default': 0.0, 'doc': 'yaw offset in degrees'},
            },
            'run': 'run_rotate_relative',
        },
        'move_to_named_position': {
            'summary': 'Drive to one of the named positions listed below.',
            'params': {
                'name': {'type': str, 'required': True, 'doc': 'name of the position'},
            },
            'run': 'run_move_to_named_position',
        },
        'home': {
            'summary': 'Return to the home pose. Refused while anything is held, '
                       'so place the object down first.',
            'params': {},
            'run': 'run_home',
        },
        'open_gripper': {
            'summary': 'Open the gripper fully.',
            'params': {},
            'run': 'run_open_gripper',
        },
        'close_gripper': {
            'summary': 'Close the gripper fully.',
            'params': {},
            'run': 'run_close_gripper',
        },
        'set_gripper': {
            'summary': 'Set the gripper to a specific opening in mm.',
            'params': {
                'opening_mm': {'type': float, 'required': True, 'min': 0.0, 'doc': 'opening in mm'},
            },
            'run': 'run_set_gripper',
        },
        'pick_object': {
            'summary': 'Pick up whatever is directly below the gripper: open, come down, '
                       'close, lift back up. Position the gripper above the object first.',
            'params': {
                'descend_mm': {'type': float, 'min': 1.0, 'doc': 'how far to come down onto the object'},
                'grip_opening_mm': {'type': float, 'min': 0.0,
                                    'doc': 'opening to close to, for a wide or fragile object'},
            },
            'run': 'run_pick_object',
        },
        'place_object': {
            'summary': 'Put the held object down directly below the gripper: come down, '
                       'release, lift back up. Position the gripper above the target first.',
            'params': {
                'descend_mm': {'type': float, 'min': 1.0, 'doc': 'how far to lower the object before releasing'},
            },
            'run': 'run_place_object',
        },
        'wave': {
            'summary': 'Wave at the user by rocking the wrist back and forth. The gripper stays '
                       'where it is, so this is safe from any pose. Refused while holding anything.',
            'params': {
                'times': {'type': int, 'min': 1, 'max': 10, 'doc': 'number of back-and-forth cycles'},
                'amplitude_deg': {'type': float, 'min': 1.0, 'max': 60.0, 'doc': 'swing amplitude in degrees'},
            },
            'run': 'run_wave',
        },
        'wait': {
            'summary': 'Hold still for a moment.',
            'params': {
                'seconds': {'type': float, 'required': True, 'min': 0.0, 'doc': 'how long to wait'},
            },
            'run': 'run_wait',
        },
    }

    def __init__(self, arm: xArm, logger: logging.Logger = None, to_log: bool = True):
        self.arm = arm
        self.logger = logger
        self.to_log = to_log and logger is not None

    # ------------------------------------------------------------------
    # description: what the planner is told it can do
    # ------------------------------------------------------------------

    def describe_actions(self):
        """The action vocabulary, rendered for a planner prompt."""
        blocks = []
        for name, spec in self.ACTION_SPECS.items():
            params = spec['params']
            signature = '{}({})'.format(name, ', '.join(params)) if params else name
            lines = [signature, f'    {spec["summary"]}']
            for param_name, param_spec in params.items():
                bits = [param_spec.get('doc', '')]
                if param_spec.get('required'):
                    bits.append('required')
                elif 'default' in param_spec:
                    bits.append(f'default {param_spec["default"]}')
                else:
                    bits.append('optional, omit for the configured default')
                if 'min' in param_spec:
                    bits.append(f'min {param_spec["min"]}')
                if 'max' in param_spec:
                    bits.append(f'max {param_spec["max"]}')
                lines.append('    - {}: {}'.format(param_name, '; '.join(bit for bit in bits if bit)))
            blocks.append('\n'.join(lines))
        return '\n\n'.join(blocks)

    def system_prompt(self, extra_rules: str = '') -> str:
        """
        Build the planner system prompt from the live registry and the arm's real limits.

        The planner sees the actual envelope numbers, so it can plan a long move as a
        sequence of legal steps instead of guessing and being rejected.
        """
        state = self.arm.describe_state()
        pose = state['pose']
        prompt = f"""You are the motion planner for a laboratory robot arm.

Convert the user's request into a sequence of robot actions, chosen only from the
vocabulary below.

AVAILABLE ACTIONS

{self.describe_actions()}

LIMITS (these are enforced; a plan that breaks one is rejected without moving)

{self.arm.describe_envelope()}

CURRENT STATE

  gripper at x={pose['x']}, y={pose['y']}, z={pose['z']} mm
  gripper opening {state['gripper_opening_mm']} mm
  holding an object: {state['holding_object']}

RULES

- Use only the actions listed above, with only the parameters listed above.
- Coordinates are in mm, in the arm base frame. +z is up.
- Every position you reach must stay inside the allowed volume. Work out the resulting
  coordinates as you plan, starting from the current state above.
- A translation larger than the single-step limit must be split into several
  move_relative actions.
- To pick something up: position the gripper above it with move_relative, then
  pick_object. To put it down: position the gripper above the target, then place_object.
- The arm cannot see. If the request needs a position you were not given, do not guess a
  large motion: return an empty action list and say what coordinate or offset you need.
- Place a held object down before going home.
- Prefer the fewest actions that do the job.
- If the request cannot be done with these actions, return an empty action list and
  explain why.
{extra_rules}"""
        return prompt

    def describe_state(self):
        return self.arm.describe_state()

    # ------------------------------------------------------------------
    # parsing and validation
    # ------------------------------------------------------------------

    def parse_action(self, raw):
        """
        Normalise one action into ``(name, params)``.

        Accepts a plain dict or anything with ``model_dump``/``dict`` (e.g. a pydantic
        model straight out of the planner response).
        """
        if hasattr(raw, 'model_dump'):
            raw = raw.model_dump()
        elif hasattr(raw, 'dict') and not isinstance(raw, dict):
            raw = raw.dict()
        if not isinstance(raw, dict):
            raise ValueError(f'expected an action object, got {type(raw).__name__}')

        raw = dict(raw)
        name = raw.pop('action', None)
        if name is None:
            raise ValueError("action object has no 'action' field")
        spec = self.ACTION_SPECS.get(name)
        if spec is None:
            raise ValueError(f'unknown action {name!r}; available: {sorted(self.ACTION_SPECS)}')

        # a flat planner schema carries every parameter field on every action, so an
        # unset field arriving as None is not an error -- a filled-in one is
        unknown = [key for key in raw if key not in spec['params'] and raw[key] is not None]
        if unknown:
            raise ValueError(f'action {name!r} got unexpected parameters {unknown}')

        params = {}
        for param_name, param_spec in spec['params'].items():
            value = raw.get(param_name, None)
            if value is None:
                if param_spec.get('required'):
                    raise ValueError(f'action {name!r} is missing required parameter {param_name!r}')
                if 'default' in param_spec:
                    params[param_name] = param_spec['default']
                continue
            params[param_name] = self._coerce(name, param_name, value, param_spec)
        return name, params

    @staticmethod
    def _coerce(action_name, param_name, value, param_spec):
        kind = param_spec['type']
        try:
            value = kind(value)
        except (TypeError, ValueError):
            raise ValueError(
                f'action {action_name!r}: parameter {param_name!r} must be {kind.__name__}, got {value!r}'
            )
        if kind is not str:
            low, high = param_spec.get('min'), param_spec.get('max')
            if low is not None and value < low:
                raise ValueError(f'action {action_name!r}: {param_name}={value} is below the minimum of {low}')
            if high is not None and value > high:
                raise ValueError(f'action {action_name!r}: {param_name}={value} is above the maximum of {high}')
        return value

    def _simulate(self, name, params, pose):
        """
        Advance a simulated TCP pose through one action, checking it stays legal.

        ``pose`` is None once the plan reaches an action whose outcome cannot be predicted
        here (``home`` moves in joint space). From that point the envelope is only checked
        at execution time -- ``xArm`` re-validates every single move against the live pose
        regardless, so this is a way to catch bad plans early rather than the real guard.

        :return: the simulated pose after the action, or None if it became unknown
        :raises SafetyError: if the action would break a limit
        """
        arm = self.arm

        def step(delta, from_pose):
            if from_pose is None:
                arm.check_step_size(delta)
                return None
            return arm.plan_relative_move(delta, from_pose=from_pose)

        if name == 'move_relative':
            return step([params.get('dx', 0.0), params.get('dy', 0.0), params.get('dz', 0.0)], pose)

        if name == 'rotate_relative':
            delta = [0.0, 0.0, 0.0,
                     params.get('droll', 0.0), params.get('dpitch', 0.0), params.get('dyaw', 0.0)]
            return step(delta, pose)

        if name in ('pick_object', 'place_object'):
            descend = params.get('descend_mm')
            if descend is None:
                descend = float(arm.safety['pick_place']['descend'])
            lift = arm.safety['pick_place']['lift'] or descend
            # down onto the object, then back up: check both legs
            down = step([0.0, 0.0, -descend], pose)
            return step([0.0, 0.0, float(lift)], down)

        if name == 'move_to_named_position':
            target = params['name']
            allowed = tuple(arm.safety['named_positions'])
            if target not in allowed:
                raise SafetyError(f'{target!r} is not a whitelisted named position; available: {list(allowed)}')
            declared = (list(arm.config.pos_dict[target]) + [None] * 6)[:6]
            arm.assert_pose_allowed(declared)
            if pose is None or any(value is None for value in declared):
                # what an unspecified component resolves to is arm specific (on the xArm6 a
                # None y is absorbed by the linear track), so stop predicting from here and
                # let the live checks in xArm cover the rest of the plan
                return None
            return [float(value) for value in declared]

        if name == 'wait':
            limit = float(arm.safety['max_wait_s'])
            if params['seconds'] > limit:
                raise SafetyError(f'a wait of {params["seconds"]} s exceeds the {limit} s limit')
            return pose

        if name == 'set_gripper':
            limit = float(arm.config.gripper_open_dist)
            if params['opening_mm'] > limit:
                raise SafetyError(f'gripper opening {params["opening_mm"]} mm exceeds the {limit} mm maximum')
            return pose

        if name == 'home':
            # joint-space move, so the resulting TCP pose is not predicted here
            return None

        # get_state, open_gripper, close_gripper, wave: the TCP stays where it is
        return pose

    def validate_plan(self, actions):
        """
        Check a whole plan without moving anything.

        :return: list of human-readable problems; an empty list means the plan is accepted
        """
        problems = []
        try:
            safety = self.arm.safety
        except SafetyError as error:
            return [str(error)]

        if not actions:
            return ['the plan contains no actions']
        if len(actions) > safety['max_plan_actions']:
            problems.append(
                f'the plan has {len(actions)} actions, more than the limit of {safety["max_plan_actions"]}'
            )

        try:
            pose = self.arm.get_cartesian_pos()
        except Exception as error:
            return [f'could not read the current arm pose: {error}']

        allowed, reason = self.arm.check_pose_allowed(pose)
        if not allowed:
            problems.append(
                f'the arm is currently outside its safety envelope ({reason}); '
                f'move it back by hand or run a station routine before driving it freely'
            )

        for index, raw in enumerate(actions, start=1):
            try:
                name, params = self.parse_action(raw)
            except ValueError as error:
                problems.append(f'action {index}: {error}')
                pose = None
                continue
            try:
                pose = self._simulate(name, params, pose)
            except SafetyError as error:
                problems.append(f'action {index} ({name}): {error}')
                pose = None
        return problems

    # ------------------------------------------------------------------
    # execution
    # ------------------------------------------------------------------

    def log(self, msg, level='info'):
        if self.to_log:
            log_and_print(True, self.logger, level, msg)
        if not self.to_log or level == 'info':
            print(msg)

    def run_action(self, name, **params):
        """Run a single validated action. Prefer ``execute_plan`` for anything multi-step."""
        spec = self.ACTION_SPECS.get(name)
        if spec is None:
            raise ValueError(f'unknown action {name!r}; available: {sorted(self.ACTION_SPECS)}')
        name, params = self.parse_action({'action': name, **params})
        return getattr(self, spec['run'])(**params)

    def execute_plan(self, actions, dry_run=False, confirm=False, stop_on_error=True):
        """
        Validate a plan in full, then run it.

        :param actions: list of action dicts (or pydantic models) from the planner
        :param dry_run: validate and print only, move nothing
        :param confirm: ask on stdin before the first action actually runs
        :param stop_on_error: stop the plan at the first failing action and leave the arm
            where it is, rather than carrying on with a sequence whose assumptions broke
        :return: list of per-action results
        :raises SafetyError: if validation rejected the plan (nothing has moved)
        """
        parsed = []
        for index, raw in enumerate(actions, start=1):
            try:
                parsed.append(self.parse_action(raw))
            except ValueError as error:
                parsed.append((None, {'error': str(error)}))

        print('Plan:')
        listing_limit = 25
        for index, (name, params) in enumerate(parsed, start=1):
            if index > listing_limit:
                print(f'  ... and {len(parsed) - listing_limit} more actions')
                break
            if name is None:
                print(f'  {index}. <invalid> {params["error"]}')
            else:
                print(f'  {index}. {name}({", ".join(f"{k}={v}" for k, v in params.items())})')

        problems = self.validate_plan(actions)
        if problems:
            message = 'plan rejected, nothing moved:\n  - ' + '\n  - '.join(problems)
            self.log(message, level='error')
            raise SafetyError(message)
        print('\nValidation passed: every step stays inside the safety envelope.')

        if dry_run:
            print('Dry run, nothing executed.')
            return [{'action': name, 'params': params, 'ok': None} for name, params in parsed]

        if confirm:
            answer = input('Execute this plan on the real arm? type yes to continue: ').strip().lower()
            if answer != 'yes':
                print('Cancelled, nothing executed.')
                return []

        self.log(f'executing a {len(parsed)}-action plan')
        results = []
        for index, (name, params) in enumerate(parsed, start=1):
            self.log(f'[{index}/{len(parsed)}] {name}({params})')
            try:
                result = getattr(self, self.ACTION_SPECS[name]['run'])(**params)
            except Exception as error:
                self.log(f'[{index}/{len(parsed)}] {name} failed: {error}', level='error')
                results.append({'action': name, 'params': params, 'ok': False, 'error': str(error)})
                if stop_on_error:
                    self.log('plan stopped; the arm has been left where it is', level='warning')
                    break
                continue
            results.append({'action': name, 'params': params, 'ok': True, 'result': result})
        return results

    # ------------------------------------------------------------------
    # the actions themselves
    # ------------------------------------------------------------------

    def run_get_state(self):
        return self.arm.describe_state()

    def run_move_relative(self, dx=0.0, dy=0.0, dz=0.0):
        return self.arm.move_relative(dx=dx, dy=dy, dz=dz)

    def run_rotate_relative(self, droll=0.0, dpitch=0.0, dyaw=0.0):
        return self.arm.move_relative(droll=droll, dpitch=dpitch, dyaw=dyaw)

    def run_move_to_named_position(self, name):
        return self.arm.move_to_named_position(name)

    def run_home(self):
        return self.arm.safe_home()

    def run_open_gripper(self):
        self.arm.open_gripper()
        return {'gripper_opening_mm': round(self.arm.gripper_opening, 2)}

    def run_close_gripper(self):
        self.arm.close_gripper()
        return {'gripper_opening_mm': round(self.arm.gripper_opening, 2)}

    def run_set_gripper(self, opening_mm):
        return {'gripper_opening_mm': self.arm.set_gripper_opening(opening_mm)}

    def run_pick_object(self, descend_mm=None, grip_opening_mm=None):
        return self.arm.pick_object(descend=descend_mm, grip_opening=grip_opening_mm)

    def run_place_object(self, descend_mm=None):
        return self.arm.place_object(descend=descend_mm)

    def run_wave(self, times=None, amplitude_deg=None):
        return self.arm.wave(times=times, amplitude_deg=amplitude_deg)

    def run_wait(self, seconds):
        sleep(min(float(seconds), float(self.arm.safety['max_wait_s'])))
        return {'waited_s': seconds}


class RobotTest_VLA(RobotTest):
    """
    Arm-only harness for free-form / VLA-driven control.

    The electrochemistry stack (EC-Lab, camera, database) is optional here: driving the
    arm from a text instruction does not need any of it. ``self.vla`` is the validated
    action surface a planner talks to.
    """

    def __init__(
            self,
            exp_name: str,
            arm: xArm,
            ec_lab: EC_Lab = None,
            camera: CameraControl = None,
            db: Database = None,
            to_log=True,
            prompt_window_bypass_list: List[Callable[[], None]] = None,
            *args,
            **kwargs
    ):
        super().__init__(
            exp_name=exp_name,
            ec_lab=ec_lab,
            camera=camera,
            arm=arm,
            db=db,
            to_log=to_log,
            prompt_window_bypass_list=prompt_window_bypass_list or [],
            *args,
            **kwargs
        )
        self.vla = VLARobot(arm, logger=self.logger, to_log=to_log)

    def test_main_workflow(self, *args, **kwargs):
        raise NotImplementedError(
            'RobotTest_VLA drives the arm directly; use rt.vla.execute_plan(...) '
            'or one of the platform harnesses for an electrochemical test'
        )

    def robot_warming_up(self):
        self.arm.safe_home()

    def robot_returning_home(self):
        self.arm.safe_home()

    def check_test_finished(self):
        return True
