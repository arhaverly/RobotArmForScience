#!/usr/bin/env python3
"""
Drive the robot arm from a plain-English instruction.

Run it from anywhere:

    python vla.py                                  # interactive, connects to the real arm
    python vla.py "wave at me"                     # one instruction, then exit
    python vla.py --simulate                       # no robot involved, safe to explore
    python vla.py --simulate "pick up the object in front of you"
    python vla.py --plan '[{"action": "wave", "times": 2}]'   # skip the planner entirely

Nothing moves without showing you the plan and asking first. Add --dry-run to plan and
validate without ever moving, or --yes to skip the confirmation prompt.
"""
import argparse
import json
import os
import sys

# --- make `python vla.py` work from any directory ---------------------------
# The lab modules are imported as `robotic_testing.*`, `utils.*` and so on, which means
# the repository root has to be importable. Running a script only puts the script's own
# directory on sys.path, so we add the root here rather than asking anyone to remember
# `python -m robotic_testing.vla` or to set PYTHONPATH.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


class Problem(Exception):
    """Something the person running this can fix. Printed without a traceback."""


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


def make_planner(vla, model):
    """Build the instruction -> plan function, or raise Problem explaining what is missing."""
    try:
        from dotenv import load_dotenv
        from google import genai
        from google.genai import types
        from pydantic import BaseModel, Field
        from typing import List, Literal, Optional
    except ImportError as error:
        raise explain_missing_module(error)

    load_dotenv(os.path.join(REPO_ROOT, '.env'))
    load_dotenv()
    api_key = os.environ.get('GEMINI_API_KEY')
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

    def plan(instruction):
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
    parser.add_argument('--model', default='gemini-3.5-flash',
                        help='planner model (default: gemini-3.5-flash)')
    args = parser.parse_args(argv)

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

    planner = make_planner(vla, args.model)

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
