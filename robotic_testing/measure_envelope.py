#!/usr/bin/env python3
"""
Derive a safety envelope from where the arm can actually go.

The envelope in each arm config bounds every free-form move. Guessing it from the
station coordinates in ``pos_dict`` is not good enough: ``home`` is defined in joint
space, so its TCP pose is not written down anywhere and is easy to leave out -- which
is exactly how an envelope ends up excluding the arm's own resting position.

This tool only ever READS the arm. It never commands a motion. You move the arm --
by hand, with the teach pendant, or with the existing station routines -- and press
Enter at each pose you want inside the envelope. At the end it prints a safety_dict
block to paste into the arm config.

    python measure_envelope.py                 # the real arm
    python measure_envelope.py --arm xarm6
    python measure_envelope.py --simulate      # to see what it does

Poses worth recording, at minimum:
  * home
  * each station the arm visits (rack corners, flask, rinsing, mid-station)
  * the highest and lowest points you are happy for it to reach
"""
import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def connect(simulate, arm_kind):
    if simulate:
        from robotic_testing.common.robotic_arms.xarm_sim import make_sim_arm
        print('Simulated arm: the numbers below are made up, this is only a demonstration.\n')
        return make_sim_arm(arm_kind)
    if arm_kind == 'xarm7':
        from robotic_testing.common.robotic_arms.xarm7.xarm7 import xArm7
        return xArm7()
    from robotic_testing.common.robotic_arms.xarm6.xarm6 import xArm6
    return xArm6()


def render(poses, margin, floor_margin, arm_kind):
    xs = [pose[0] for pose in poses]
    ys = [pose[1] for pose in poses]
    zs = [pose[2] for pose in poses]
    reaches = [(pose[0] ** 2 + pose[1] ** 2) ** 0.5 for pose in poses]

    # the z floor gets a smaller margin: down is where the bench is
    lines = [
        '',
        '=' * 72,
        f'Measured {len(poses)} pose(s). Bounding box of what you recorded:',
        f'    x: {min(xs):8.1f} .. {max(xs):8.1f}',
        f'    y: {min(ys):8.1f} .. {max(ys):8.1f}',
        f'    z: {min(zs):8.1f} .. {max(zs):8.1f}',
        f'    horizontal reach: {min(reaches):.1f} .. {max(reaches):.1f}',
        '',
        f'With a {margin:.0f} mm margin ({floor_margin:.0f} mm below the lowest point),',
        f'paste this into safety_dict in the {arm_kind} config:',
        '',
        f"    'x_range': ({min(xs) - margin:.1f}, {max(xs) + margin:.1f}),",
        f"    'y_range': ({min(ys) - margin:.1f}, {max(ys) + margin:.1f}),",
        f"    'z_range': ({min(zs) - floor_margin:.1f}, {max(zs) + margin:.1f}),",
        f"    'max_reach_mm': {max(reaches) + margin:.1f},",
        '',
        'Check it before trusting it: the box is the volume free-form motion may use, so',
        'anything inside it that the arm must not hit needs the range pulled back instead.',
        '=' * 72,
    ]
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--arm', default='xarm7', choices=['xarm6', 'xarm7'])
    parser.add_argument('--simulate', action='store_true', help='no robot, just show the flow')
    parser.add_argument('--margin', type=float, default=50.0,
                        help='mm of slack around the measured box (default: 50)')
    parser.add_argument('--floor-margin', type=float, default=20.0,
                        help='mm of slack below the lowest measured point (default: 20)')
    args = parser.parse_args()

    arm = connect(args.simulate, args.arm)

    print('This tool never moves the arm. Move it yourself, then record each pose.')
    print('  Enter      record the current pose')
    print('  d          drop the last recorded pose')
    print('  q          finish and print the envelope\n')

    poses = []
    while True:
        try:
            answer = input(f'[{len(poses)} recorded] Enter to record, q to finish> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if answer in ('q', 'quit', 'exit'):
            break
        if answer == 'd':
            if poses:
                dropped = poses.pop()
                print(f'  dropped x={dropped[0]:.1f} y={dropped[1]:.1f} z={dropped[2]:.1f}')
            else:
                print('  nothing to drop')
            continue

        pose = arm.get_cartesian_pos()
        poses.append(pose)
        print(f'  recorded x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f}  '
              f'(roll={pose[3]:.1f} pitch={pose[4]:.1f} yaw={pose[5]:.1f})')

    if not poses:
        print('No poses recorded, so there is nothing to compute.')
        return 1

    print(render(poses, args.margin, args.floor_margin, args.arm))
    return 0


if __name__ == '__main__':
    sys.exit(main())
