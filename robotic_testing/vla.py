#!/usr/bin/env python3
"""
Drive the robot arm from a plain-English instruction.

Run it from anywhere:

    python vla.py                                  # interactive, connects to the real arm
    python vla.py "wave at me"                     # one instruction, then exit
    python vla.py --simulate                       # no robot involved, safe to explore
    python vla.py --simulate "pick up the object in front of you"
    python vla.py --plan '[{"action": "wave", "times": 2}]'   # skip the planner entirely
    python vla.py --model gemini-3.8-flash          # a specific planner model
    python vla.py --list-models                    # what this API key can plan with

When a model is overloaded -- Google answers 503 and the message says the model is
busy -- the planner waits, asks again, and then moves down a chain of models rather
than giving up. --model overrides the chain.

Nothing moves without showing you the plan and asking first. Add --dry-run to plan and
validate without ever moving, or --yes to skip the confirmation prompt.
"""
import argparse
import json
import os
import re
import sys
import time

# --- make `python vla.py` work from any directory ---------------------------
# The lab modules are imported as `robotic_testing.*`, `utils.*` and so on, which means
# the repository root has to be importable. Running a script only puts the script's own
# directory on sys.path, so we add the root here rather than asking anyone to remember
# `python -m robotic_testing.vla` or to set PYTHONPATH.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# A venv built from a conda interpreter on Windows cannot import ssl or sqlite3 until
# its base DLL directories are added to the search path, which means no planner and no
# pip. Importing this does that and is a no-op on every other platform; it sits here,
# above the third-party imports, because it has to run before the first `import ssl`.
from robotic_testing.common import windows_dlls  # noqa: F401,E402


# The planner is asked for a plan in this order, moving to the next name when one is
# overloaded. Google returns 503 UNAVAILABLE for "the model is overloaded" and that is
# per-model capacity, so a chain is worth more than retrying one name harder -- and the
# last entry is deliberately a different generation rather than a neighbouring Flash.
DEFAULT_MODELS = ('gemini-3.5-flash', 'gemini-3.6-flash', 'gemini-2.5-flash')

ATTEMPTS_PER_MODEL = 3    # then move to the next model in the chain
BACKOFF_S = 2.0           # doubling: 2s, 4s

# HTTP statuses worth waiting out. 429 is a rate limit, 503 is the overload everyone
# hits, 500/504 are transient server-side faults.
TRANSIENT_STATUSES = frozenset((429, 500, 502, 503, 504))
# Statuses where trying another model cannot help: the key or the request is the problem.
FATAL_STATUSES = frozenset((400, 401, 403))


class Problem(Exception):
    """Something the person running this can fix. Printed without a traceback."""


def api_status(error):
    """The HTTP status behind a google-genai exception, or None if it has none."""
    for attribute in ('code', 'status_code'):
        value = getattr(error, attribute, None)
        if isinstance(value, int):
            return value
    response = getattr(error, 'response', None)
    value = getattr(response, 'status_code', None)
    if isinstance(value, int):
        return value
    # Older SDKs put the status only in the text. Match a bare 3-digit code so a
    # model name or a token count cannot be mistaken for one.
    match = re.search(r'\b(4\d\d|5\d\d)\b', str(error))
    return int(match.group(1)) if match else None


def classify_api_error(error):
    """
    'transient' (wait and retry), 'skip' (try another model), or 'fatal' (stop).

    Status first, because it is unambiguous; wording second, because the SDK does not
    always carry one.
    """
    status = api_status(error)
    if status in TRANSIENT_STATUSES:
        return 'transient'
    if status in FATAL_STATUSES:
        return 'fatal'
    if status == 404:
        return 'skip'           # no such model for this key; another name may exist
    text = str(error).lower()
    if any(word in text for word in
           ('overload', 'unavailable', 'try again', 'busy', 'rate limit', 'exhausted',
            'deadline', 'timeout', 'temporarily')):
        return 'transient'
    if any(word in text for word in ('api key', 'permission', 'unauthenticated', 'quota')):
        return 'fatal'
    return 'skip'


def explain_api_error(error, model):
    """A Problem for something no amount of retrying or model-swapping will fix."""
    status = api_status(error)
    if status in (401, 403) or 'api key' in str(error).lower():
        return Problem(
            'The Gemini API rejected the key ({}).\n'
            '  Check GEMINI_API_KEY in the .env file at the repository root.\n'
            '  Or skip the planner entirely:  python vla.py --plan \'[{{"action": "wave"}}]\''
            .format(error))
    if 'quota' in str(error).lower():
        return Problem(
            'The Gemini API key is out of quota, so no model will answer ({}).\n'
            '  Check the quota for the key, or use --plan to drive the arm without '
            'the planner.'.format(error))
    return Problem(
        'The planner request was rejected by {} and retrying will not change that:\n'
        '  {}\n'
        '  See which models this key can use:  python vla.py --list-models'
        .format(model, error))


def explain_missing_module(error):
    """Turn an ImportError from the lab stack into something actionable."""
    name = getattr(error, 'name', '') or str(error)
    root = name.split('.')[0]

    if root == 'xarm':
        return Problem(
            'The xArm SDK is not installed, so there is no way to reach the robot.\n'
            '  Install it with:  pip install xarm-python-sdk\n'
            '  Or explore without a robot:  python vla.py --simulate'
        )
    if root == 'sensitives' or 'sensitives' in name:
        return Problem(
            'utils/sensitives.py is missing. It is deliberately kept out of git because it\n'
            'holds credentials, so a fresh clone never has it. Copy it from the lab PC,\n'
            'or explore without it:  python vla.py --simulate'
        )
    if root in ('google', 'dotenv'):
        return Problem(
            f'The planner needs a package that is not installed ({root}).\n'
            f'  Install the planner dependencies:  pip install -r robotic_testing/requirements-vla.txt\n'
            f'  Or skip the planner and pass a plan directly with --plan'
        )
    # A stdlib C extension that will not load is not a missing package, and no amount
    # of pip will help -- pip itself needs _ssl to reach the network, so the advice
    # below would send someone in a circle. This is the conda-venv DLL problem; see
    # robotic_testing/common/windows_dlls.py.
    if root.startswith('_') or 'DLL load failed' in str(error):
        return Problem(
            f'This Python cannot import part of its own standard library ({root}).\n'
            f'That is an interpreter problem, not a missing package -- pip cannot fix it,\n'
            f'and on a conda-based venv pip will fail the same way.\n'
            f'  Repair this environment:  python robotic_testing/setup_windows_env.py\n'
            f'  See what is broken first:  python robotic_testing/setup_windows_env.py --check'
        )
    return Problem(
        f'A required package is not installed: {root}\n'
        f'  Install the dependencies:  pip install -r robotic_testing/requirements-vla.txt\n'
        f'  Or explore without a robot:  python vla.py --simulate'
    )


def connect_arm(simulate, arm_kind):
    """Return (arm, vla). Raises Problem with a readable message if something is missing."""
    try:
        if simulate:
            from robotic_testing.common.robotic_arms.xarm_sim import make_sim_arm
            print(f'Simulated {arm_kind}: no robot is connected and nothing will physically move.\n')
            arm = make_sim_arm(arm_kind)
        else:
            if arm_kind == 'xarm7':
                from robotic_testing.common.robotic_arms.xarm7.xarm7 import xArm7 as arm_class
            else:
                from robotic_testing.common.robotic_arms.xarm6.xarm6 import xArm6 as arm_class
            print(f'Connecting to the real {arm_kind}...')
            arm = arm_class()

        from robotic_testing.common.vla_control import VLARobot
    except ImportError as error:
        raise explain_missing_module(error)

    return arm, VLARobot(arm, logger=None, to_log=False)


def show_state(arm):
    state = arm.describe_state()
    pose = state['pose']
    print('Arm state')
    print(f"  gripper at  x={pose['x']}  y={pose['y']}  z={pose['z']}  (mm)")
    print(f"  gripper opening  {state['gripper_opening_mm']} mm")
    print(f"  holding an object  {state['holding_object']}")
    if state.get('inside_envelope') is False:
        print(f"  WARNING: outside the safety envelope -- {state.get('envelope_violation')}")
        print('  Free-form motion will be refused until the arm is back inside it.')
    print()
    print(arm.describe_envelope())
    print()
    return state


def load_api_key():
    """GEMINI_API_KEY, from the repo's .env or the environment. None if unset."""
    from dotenv import load_dotenv
    load_dotenv(os.path.join(REPO_ROOT, '.env'))
    load_dotenv()
    return os.environ.get('GEMINI_API_KEY')


def list_models():
    """
    Print the models this key can actually generate content with.

    Worth having because a wrong model name and an overloaded model produce very
    different failures, and guessing at names from documentation is how one gets
    mistaken for the other.
    """
    try:
        from google import genai
    except ImportError as error:
        raise explain_missing_module(error)

    api_key = load_api_key()
    if not api_key:
        raise Problem('GEMINI_API_KEY is not set, so the API cannot be asked what it offers.')

    try:
        models = list(genai.Client(api_key=api_key).models.list())
    except Exception as error:                      # noqa: BLE001 -- report, never traceback
        raise explain_api_error(error, 'models.list')

    usable = []
    for model in models:
        actions = getattr(model, 'supported_actions', None)
        if actions and 'generateContent' not in actions:
            continue
        usable.append(str(getattr(model, 'name', model)).replace('models/', ''))

    print('Models this key can plan with:')
    for name in sorted(usable):
        mark = '  <-- in the default chain' if name in DEFAULT_MODELS else ''
        print('  {}{}'.format(name, mark))
    print('\nUse one with:  python vla.py --model NAME')
    print('Or a fallback chain:  python vla.py --model NAME,OTHER,THIRD')
    return 0


def make_planner(vla, models=None):
    """
    Build the instruction -> plan function, or raise Problem explaining what is missing.

    `models` is one name, a list of them tried in order, or None for the default chain
    -- None being the useful default from a notebook, where nobody wants to name a model
    to ask a question.
    """
    try:
        from dotenv import load_dotenv
        from google import genai
        from google.genai import types
        from pydantic import BaseModel, Field
        from typing import List, Literal, Optional
    except ImportError as error:
        raise explain_missing_module(error)

    api_key = load_api_key()
    if not api_key:
        raise Problem(
            'GEMINI_API_KEY is not set, so the planner cannot be reached.\n'
            '  Put it in a .env file at the repository root:  GEMINI_API_KEY=your-key-here\n'
            '  Or export it:  export GEMINI_API_KEY=your-key-here\n'
            '  Or skip the planner and pass a plan directly, e.g.\n'
            '    python vla.py --plan \'[{"action": "wave", "times": 2}]\''
        )

    # one flat action model covering every parameter in the registry; the executor is
    # what actually type- and range-checks each action, so this only has to carry values
    action_names = tuple(vla.ACTION_SPECS)
    fields = {}
    for spec in vla.ACTION_SPECS.values():
        for param_name, param_spec in spec['params'].items():
            fields[param_name] = (Optional[param_spec['type']],
                                  Field(default=None, description=param_spec.get('doc', '')))

    from pydantic import create_model
    RobotAction = create_model('RobotAction', action=(Literal[action_names], ...), **fields)
    RobotPlan = create_model(
        'RobotPlan',
        reasoning=(str, Field(description='Brief explanation of the intended procedure.')),
        actions=(List[RobotAction], ...),
    )

    client = genai.Client(api_key=api_key)

    if models is None:
        chain = list(DEFAULT_MODELS)
    elif isinstance(models, str):
        chain = [models]
    else:
        chain = [name for name in models if name] or list(DEFAULT_MODELS)

    def ask(model, instruction):
        response = client.models.generate_content(
            model=model,
            contents=f'{vla.system_prompt()}\n\nUSER REQUEST:\n{instruction}\n',
            config=types.GenerateContentConfig(
                response_mime_type='application/json',
                response_schema=RobotPlan,
                temperature=0,
            ),
        )
        return RobotPlan.model_validate_json(response.text)

    def plan(instruction):
        """
        Ask each model in the chain, waiting out the ones that are merely busy.

        Nothing here changes what the arm does: a plan still comes back, or a Problem
        explaining why none did. The arm is not touched either way, so retrying is
        free of consequence -- the only cost is the operator's time, which is why the
        waiting is announced rather than silent.
        """
        refused = []
        for position, model in enumerate(chain):
            for attempt in range(ATTEMPTS_PER_MODEL):
                try:
                    plan_object = ask(model, instruction)
                except Exception as error:          # noqa: BLE001 -- classified below
                    kind = classify_api_error(error)
                    if kind == 'fatal':
                        raise explain_api_error(error, model)
                    last = attempt + 1 >= ATTEMPTS_PER_MODEL
                    if kind == 'transient' and not last:
                        delay = BACKOFF_S * (2 ** attempt)
                        print(f'  {model} is busy; waiting {delay:.0f}s and asking again '
                              f'({attempt + 2} of {ATTEMPTS_PER_MODEL})')
                        time.sleep(delay)
                        continue
                    refused.append((model, str(error).strip().splitlines()[0]))
                    if position + 1 < len(chain):
                        print(f'  {model} would not answer; trying {chain[position + 1]}')
                    break
                else:
                    if position:
                        print(f'  planned with {model}')
                    return plan_object

        lines = '\n'.join(f'  {model}: {why}' for model, why in refused)
        # A 429 is retried and fallen through rather than treated as fatal, because
        # Gemini quotas are per-model and the next name in the chain often has room.
        # If that is what happened, say so: telling someone to wait out Google's
        # capacity when it is their own key's quota sends them to the wrong place.
        cause = ('Every one of those is a quota limit on this key, not Google being '
                 'busy.\n'
                 if all('quota' in why.lower() or 'exhausted' in why.lower()
                        for _, why in refused)
                 else 'If they are all busy, this is Google\'s capacity and waiting is '
                      'the only cure.\n')
        raise Problem(
            'No planner model would answer. Tried:\n{}\n\n'
            '{}'
            '  Name a different model:  python vla.py --model NAME\n'
            '  See what this key can use:  python vla.py --list-models\n'
            '  Or drive the arm without the planner:\n'
            '    python vla.py --plan \'[{{"action": "wave", "times": 2}}]\''
            .format(lines, cause))

    return plan


def run_plan(vla, actions, dry_run, assume_yes):
    """Hand a list of action dicts to the validating executor."""
    from robotic_testing.common.robotic_arms.xarm_control import SafetyError
    try:
        return vla.execute_plan(actions, dry_run=dry_run, confirm=not (dry_run or assume_yes))
    except SafetyError:
        # execute_plan has already printed why each action was rejected
        print('\nNothing moved. Reword the instruction, or widen the envelope in the arm '
              'config if the move really is safe.')
        return []


def handle_instruction(vla, planner, instruction, dry_run, assume_yes):
    print(f'\n> {instruction}')
    plan = planner(instruction)
    print(f'\nPlanner: {plan.reasoning}\n')
    if not plan.actions:
        print('The planner produced no actions -- see its explanation above.')
        return
    actions = [action.model_dump(exclude_none=True) for action in plan.actions]
    run_plan(vla, actions, dry_run, assume_yes)


def interactive(vla, planner, dry_run, assume_yes):
    print('Type an instruction, or "state" to re-read the arm, or "quit" to exit.')
    while True:
        try:
            instruction = input('\ninstruction> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not instruction:
            continue
        if instruction.lower() in ('quit', 'exit', 'q'):
            return
        if instruction.lower() in ('state', 'status'):
            show_state(vla.arm)
            continue
        try:
            handle_instruction(vla, planner, instruction, dry_run, assume_yes)
        except Problem:
            raise
        except Exception as error:
            print(f'\nThat instruction failed: {error}')
            print('The arm has been left where it is.')


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Drive the robot arm from a plain-English instruction.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('instruction', nargs='?',
                        help='what to do, e.g. "wave at me". Omit for an interactive prompt.')
    parser.add_argument('--simulate', action='store_true',
                        help='use a simulated arm; no robot, nothing physically moves')
    parser.add_argument('--arm', default='xarm7', choices=['xarm6', 'xarm7'],
                        help='which arm to drive (default: xarm7, the 270 platform)')
    parser.add_argument('--dry-run', action='store_true',
                        help='plan and validate only, never move')
    parser.add_argument('--yes', action='store_true',
                        help='do not ask for confirmation before moving')
    parser.add_argument('--plan', metavar='JSON',
                        help='run this JSON action list directly, without calling the planner')
    parser.add_argument('--state', action='store_true',
                        help='print the arm state and safety envelope, then exit')
    parser.add_argument('--model', action='append', metavar='NAME', default=None,
                        help='planner model, or a comma-separated chain tried in order when '
                             'one is overloaded. Repeat the flag to append. '
                             '(default: %s)' % ','.join(DEFAULT_MODELS))
    parser.add_argument('--list-models', action='store_true',
                        help='ask the API which models this key can plan with, then exit')
    args = parser.parse_args(argv)

    # Before connecting: this asks Google a question and has nothing to do with the arm.
    if args.list_models:
        return list_models()

    models = []
    for value in args.model or []:
        models += [name.strip() for name in value.split(',') if name.strip()]
    models = models or list(DEFAULT_MODELS)

    arm, vla = connect_arm(args.simulate, args.arm)
    show_state(arm)

    if args.state:
        return 0

    if args.plan:
        try:
            actions = json.loads(args.plan)
        except json.JSONDecodeError as error:
            raise Problem(f'--plan is not valid JSON: {error}')
        if not isinstance(actions, list):
            raise Problem('--plan must be a JSON list of actions, e.g. \'[{"action": "wave"}]\'')
        run_plan(vla, actions, args.dry_run, args.yes)
        return 0

    planner = make_planner(vla, models)

    if args.instruction:
        handle_instruction(vla, planner, args.instruction, args.dry_run, args.yes)
    else:
        interactive(vla, planner, args.dry_run, args.yes)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Problem as problem:
        print(f'\n{problem}\n', file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print('\nInterrupted. The arm has been left where it is.', file=sys.stderr)
        sys.exit(130)
