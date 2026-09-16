#!/usr/bin/env python3
"""
Put the arm in a listening state: idling constantly, stopping the instant the ESP32 speaks.

    python listen.py --list-ports                # which port is the ESP32 on?
    python listen.py --port COM7                 # the real arm, the real ESP32 (Windows)
    python listen.py --port /dev/ttyUSB0         # the same, on Linux
    python listen.py --simulate --port stdin     # no robot, no ESP32: press Enter to stop
    python listen.py --port COM7 --plan          # hand "MSG <text>" to the VLA planner

Runs on Python 3.9 and later, on Windows and on Linux. Only numpy is needed for
--simulate; a real port also needs pyserial, and a real arm the xArm SDK.

While listening, the arm wanders between random waypoints in a box around where it
started, inside the safety envelope its config declares, twitching its wrist now and
then. The moment a byte arrives from the ESP32 the arm is halted -- before the message
has even finished arriving, let alone been parsed -- and the session waits for a RESUME.

The protocol the ESP32 has to speak, and a sketch that speaks it, are in
robotic_testing/ESP32_PROTOCOL.md.
"""
import argparse
import os
import queue
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Before any third-party import: see robotic_testing/common/windows_dlls.py.
from robotic_testing.common import windows_dlls  # noqa: F401,E402

from robotic_testing.common.esp32_link import (  # noqa: E402  (needs REPO_ROOT on sys.path)
    PROTOCOL_VERSION, ESP32Link, LinkError, Trigger, describe_ports, open_transport,
)
from robotic_testing.vla import Problem, connect_arm  # noqa: E402


class ListeningSession:
    """
    The state machine: LISTENING while the arm idles, STOPPED once the ESP32 has spoken.

    The stop itself is deliberately not part of this loop. It happens on the link's reader
    thread, in :meth:`on_trigger`, so that nothing -- not the queue, not the GIL waiting on
    a print, not the move in flight -- sits between the byte and the arm halting. This loop
    only ever finds out afterwards, and its job is to explain what happened and decide
    whether to resume.
    """

    def __init__(self, arm, link, make_motion, stop_mode='hard', on_message=None,
                 status_every_s=5.0):
        self.arm = arm
        self.link = link
        self.make_motion = make_motion
        self.stop_mode = stop_mode
        self.on_message = on_message
        self.status_every_s = status_every_s

        self.state = 'STOPPED'
        self.motion = None
        self.stops = 0
        self.total_moves = 0
        self._quit = False
        self._hard_stopped = False
        self._last_status = 0.0

        # written on the reader thread, read on the main thread
        self._stop_lock = threading.Lock()
        self._stopping = False
        self._stop_event = threading.Event()
        self._trigger_at = None
        self._stop_latency = None
        self._stop_reason = None

    # ------------------------------------------------------------------
    # the stop path -- runs on the link's reader thread
    # ------------------------------------------------------------------

    def on_trigger(self, trigger):
        """
        Halt the arm. Nothing else belongs in here.

        ``xArm.emergency_stop`` puts the controller into the stop state, which abandons
        the move in flight rather than letting it finish; that is what makes this take
        milliseconds instead of the remainder of a step. It leaves the arm latched, so
        :meth:`enter_listening` clears it again before the next move.
        """
        with self._stop_lock:
            if self._stopping:
                return
            self._stopping = True

        # read once: the main thread clears self.motion as it winds the wander thread up
        motion = self.motion
        if motion is not None:
            motion.request_stop()
        if self.stop_mode == 'hard':
            try:
                self.arm.emergency_stop()
                self._hard_stopped = True
            except Exception as error:
                print(f'\n!! the emergency stop itself failed: {error}')

        self._trigger_at = trigger.at
        self._stop_latency = time.monotonic() - trigger.at
        self._stop_reason = trigger.reason
        self._stop_event.set()

    # ------------------------------------------------------------------
    # state changes -- main thread only
    # ------------------------------------------------------------------

    def enter_listening(self):
        """Clear any latched stop, start a fresh wander, and arm the trigger."""
        from robotic_testing.common.robotic_arms.xarm_control import SafetyError

        if self._hard_stopped:
            self.arm.reset_safety_state()
            self._hard_stopped = False

        try:
            motion = self.make_motion()
            motion.check_ready()
        except SafetyError as error:
            print(f'\nCannot idle: {error}')
            self.link.send('NACK', 'RESUME', str(error).split(';')[0])
            self.state = 'STOPPED'
            return False

        with self._stop_lock:
            self._stopping = False
        self._stop_event.clear()

        self.motion = motion
        motion.start()
        self.link.arm_trigger()
        self.state = 'LISTENING'
        self._last_status = time.monotonic()

        print('\nLISTENING -- the arm is moving. Anything the ESP32 sends stops it.')
        print(motion.describe())
        self.link.send('STATE', 'LISTENING')
        return True

    def _halt_motion(self):
        """Wind up the wander thread and fold its move count into the session total."""
        motion = self.motion
        self.motion = None
        if motion is None:
            return
        motion.request_stop()
        if not motion.join():
            print('  (the idle thread has not come back yet; the arm is halted regardless)')
        self.total_moves += motion.moves

    def enter_stopped(self, note=None):
        self._halt_motion()
        self.link.disarm_trigger()
        self.state = 'STOPPED'
        # announced rather than left to be inferred: the firmware uses this to know it
        # should stop heartbeating, and the periodic tick only runs while listening
        self.link.send('STATE', self.state)
        if note:
            print(note)
        pose = self.arm.get_cartesian_pos()
        print(f'STOPPED at x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f} mm. '
              f'Send RESUME to carry on, or press Ctrl-C to quit.')

    def report_stop(self):
        """
        Turn the stop that already happened into something the operator and ESP32 see.

        The number both of them get is milliseconds from the byte landing to the arm
        actually being still. That is a different measurement in the two stop modes: a
        hard stop is done once the controller has taken the command, whereas a soft one is
        only done once the step already in flight has run out.
        """
        self.stops += 1
        requested_ms = (self._stop_latency or 0.0) * 1000
        print(f'\n*** STOP -- {self._stop_reason}')
        self._halt_motion()
        if self.stop_mode == 'hard':
            halted_ms = requested_ms
            note = f'    the controller was halted {halted_ms:.2f} ms after the trigger'
        else:
            halted_ms = (time.monotonic() - (self._trigger_at or time.monotonic())) * 1000
            note = (f'    the stop was requested {requested_ms:.2f} ms after the trigger; '
                    f'the arm came to rest at {halted_ms:.0f} ms')
        self.enter_stopped(note=note)
        self.link.send('STOPPED', f'{halted_ms:.2f}')
        self._stop_event.clear()

    # ------------------------------------------------------------------
    # message handling
    # ------------------------------------------------------------------

    def handle(self, message):
        # The invariant this whole script exists to provide: no message is ever acted on
        # while the arm is still moving. The reader thread has normally stopped it long
        # before this runs, and this covers the one gap it cannot -- a message that landed
        # in the moment between the link opening and the trigger being armed.
        if self.state == 'LISTENING':
            self.on_trigger(Trigger(message.received_at, 'a message arrived over USB'))
            self.report_stop()

        verb = message.verb

        if verb in ('STOP', 'HALT', 'ESTOP'):
            self.link.send('ACK', verb)

        elif verb in ('RESUME', 'GO', 'START'):
            if self.enter_listening():
                self.link.send('ACK', verb)

        elif verb == 'HELLO':
            print(f'\nESP32 booted: {message.text or "(no version given)"}')
            self.link.send('READY', PROTOCOL_VERSION, self._arm_name())
            self.enter_listening()

        elif verb == 'PING':
            self.link.send('PONG')
            print('  (PING also stops the arm -- only the ~ heartbeat does not)')

        elif verb in ('STATE', 'POSE'):
            pose = self.arm.get_cartesian_pos()
            self.link.send('POSE', f'{pose[0]:.1f}', f'{pose[1]:.1f}', f'{pose[2]:.1f}')
            self.link.send('STATE', self.state)

        elif verb == 'HOME':
            self.go_home()

        elif verb == 'MSG':
            self.link.send('ACK', 'MSG')
            if self.on_message is not None:
                self.on_message(message.text)
            else:
                print(f'  message: {message.text}')
                print('  (nothing is wired up to act on it -- pass --plan to send it to the planner)')

        elif verb in ('QUIT', 'BYE', 'EXIT'):
            self.link.send('ACK', verb)
            self._quit = True

        elif verb == 'LINKLOST':
            print(f'\nThe link to the ESP32 went quiet ({message.text}). '
                  f'Check the cable and the board, then send RESUME.')

        elif verb == 'GARBAGE':
            print(f'\nA line arrived with no newline in sight, so it was dropped: {message.raw!r}')
            print('  Check the baud rate matches on both ends.')
            self.link.send('NACK', 'GARBAGE', 'check-baud-rate')

        else:
            print(f'\nThe ESP32 said {message.raw!r}, which is not part of the protocol.')
            self.link.send('NACK', verb, 'unknown-verb')

    def go_home(self):
        from robotic_testing.common.robotic_arms.xarm_control import SafetyError
        if self._hard_stopped:
            self.arm.reset_safety_state()
            self._hard_stopped = False
        try:
            print('\nReturning home...')
            self.arm.safe_home()
        except (SafetyError, SystemError) as error:
            print(f'Could not go home: {error}')
            self.link.send('NACK', 'HOME', str(error).split(';')[0])
            return
        self.link.send('ACK', 'HOME')
        print('Home. Send RESUME to start listening again.')

    # ------------------------------------------------------------------
    # the loop
    # ------------------------------------------------------------------

    def _arm_name(self):
        return type(self.arm).__name__.lower()

    def _status(self):
        now = time.monotonic()
        if self.status_every_s <= 0 or now - self._last_status < self.status_every_s:
            return
        self._last_status = now
        pose = self.arm.get_cartesian_pos()
        print(f'  listening... {self.motion.moves} moves, '
              f'at x={pose[0]:.0f} y={pose[1]:.0f} z={pose[2]:.0f} mm, '
              f'{self.link.heartbeats} heartbeats')
        # the ESP32 heartbeat tells the host the board is alive; this is the same promise
        # in the other direction, so the firmware can run its own watchdog on the host
        self.link.send('STATE', self.state)
        self.link.send('POSE', f'{pose[0]:.1f}', f'{pose[1]:.1f}', f'{pose[2]:.1f}')

    def run(self, deadline=None):
        print(f'\nLink: {self.link.transport.description}')
        self.link.send('READY', PROTOCOL_VERSION, self._arm_name())
        self.enter_listening()

        while not self._quit:
            if deadline is not None and time.monotonic() > deadline:
                print('\nTime limit reached.')
                break

            try:
                message = self.link.messages.get(timeout=0.2)
            except queue.Empty:
                message = None

            # the stop happened before this message was parsed, so it gets reported first
            # -- otherwise the log reads as though the arm carried on until the ESP32 was
            # answered, which is the opposite of what the link guarantees
            if self._stop_event.is_set():
                self.report_stop()

            # the wander can also end by itself: a fault, or a run of moves the arm
            # could not actually reach
            if self.state == 'LISTENING' and self.motion is not None and not self.motion.running:
                error = self.motion.error
                self.enter_stopped(note=f'\nThe idle motion stopped on its own: {error}'
                                   if error else '\nThe idle motion ended.')

            if message is None:
                if self.state == 'LISTENING':
                    self._status()
                continue
            self.handle(message)

        self.shutdown()

    def shutdown(self):
        """Leave the arm still and where it is, and tell the ESP32 we have gone."""
        if self.motion is not None and self.motion.running:
            self.motion.request_stop()
            self.motion.join()
            self.total_moves += self.motion.moves
        self.link.disarm_trigger()
        self.link.send('BYE')
        print(f'\nDone: {self.total_moves} idle moves, {self.stops} stop(s). '
              f'The arm has been left where it is.')


# ----------------------------------------------------------------------
# wiring
# ----------------------------------------------------------------------

def make_planner_handler(vla, model):
    """Route ``MSG <text>`` to the VLA planner and run what it comes back with."""
    from robotic_testing.vla import make_planner, run_plan
    planner = make_planner(vla, model)

    def handle(text):
        if not text:
            print('  MSG arrived with no text, so there is nothing to plan.')
            return
        print(f'\n> {text}')
        try:
            plan = planner(text)
        except Exception as error:
            print(f'The planner failed: {error}')
            return
        print(f'\nPlanner: {plan.reasoning}\n')
        if not plan.actions:
            print('The planner produced no actions -- see its explanation above.')
            return
        run_plan(vla, [action.model_dump(exclude_none=True) for action in plan.actions],
                 dry_run=False, assume_yes=True)

    return handle


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Idle the arm until the ESP32 says stop.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--port', default='stdin',
                        help='serial port of the ESP32, e.g. COM7 or /dev/ttyUSB0; '
                             '--list-ports shows them. The default, "stdin", uses the '
                             'keyboard instead.')
    parser.add_argument('--baud', type=int, default=115200, help='serial baud rate (default: 115200)')
    parser.add_argument('--list-ports', action='store_true',
                        help='print the serial ports this machine can see, then exit')
    parser.add_argument('--simulate', action='store_true',
                        help='use a simulated arm; no robot, nothing physically moves')
    parser.add_argument('--arm', default='xarm7', choices=['xarm6', 'xarm7'],
                        help='which arm to drive (default: xarm7, the 270 platform)')
    parser.add_argument('--stop-mode', default='hard', choices=['hard', 'soft'],
                        help='hard halts the controller mid-move in milliseconds and needs a '
                             'reset to resume; soft lets the step in flight finish first '
                             '(default: hard)')
    parser.add_argument('--radius', type=float, default=150.0,
                        help='half-width of the box the arm wanders in, mm (default: 150)')
    parser.add_argument('--step', type=float, default=40.0,
                        help='length of a single idle move, mm (default: 40)')
    parser.add_argument('--speed-profile', default=None,
                        help='pos_settings_dict key for idle moves '
                             '(default: the arm config free-form profile)')
    parser.add_argument('--no-wiggle', action='store_true', help='do not twist the wrist while idling')
    parser.add_argument('--heartbeat-timeout', type=float, default=1.5,
                        help='stop the arm if the ESP32 sends no ~ for this long, once it has '
                             'sent at least one, in seconds (default: 1.5)')
    parser.add_argument('--require-heartbeat', action='store_true',
                        help='demand the ~ heartbeat from the start, rather than only after '
                             'the first one arrives')
    parser.add_argument('--settle', type=float, default=1.0,
                        help='seconds of serial input to discard after opening the port, so a '
                             'boot banner is not read as a stop request (default: 1.0)')
    parser.add_argument('--plan', action='store_true',
                        help='hand "MSG <text>" to the VLA planner and run the resulting plan')
    parser.add_argument('--model', default='gemini-3.5-flash',
                        help='planner model, with --plan (default: gemini-3.5-flash)')
    parser.add_argument('--echo', action='store_true', help='print every line the ESP32 sends')
    parser.add_argument('--for', dest='run_for', type=float, default=None,
                        metavar='SECONDS', help='quit after this long; useful for a trial run')
    parser.add_argument('--status-every', type=float, default=5.0, metavar='SECONDS',
                        help='how often to print a status line and send STATE/POSE to the '
                             'ESP32 while listening; 0 turns both off (default: 5)')
    parser.add_argument('--seed', type=int, default=None, help='fix the wander for a repeatable demo')
    args = parser.parse_args(argv)

    # answered before the arm is touched, so it works with no robot and no ESP32
    if args.list_ports:
        print(describe_ports())
        return 0

    arm, vla = connect_arm(args.simulate, args.arm)
    if args.simulate:
        # the offline arm answers instantly, which makes a stop impossible to demonstrate;
        # give its moves a duration so there is something to interrupt
        arm.move_duration_s = 0.6

    from robotic_testing.common.idle_motion import IdleMotion
    from robotic_testing.common.robotic_arms.xarm_control import SafetyError

    # anchored once, here: if each resume re-centred the box on wherever the arm was
    # halted, a long shift of stops and resumes would walk it across the bench
    origin = arm.get_cartesian_pos()

    def make_motion():
        return IdleMotion(arm, radius_mm=args.radius, step_mm=args.step,
                          wiggle_chance=0.0 if args.no_wiggle else 0.35,
                          settings_profile=args.speed_profile, origin=origin, seed=args.seed)

    # fail before the arm starts moving, not on the first message
    on_message = make_planner_handler(vla, args.model) if args.plan else None

    try:
        transport = open_transport(args.port, baud=args.baud, settle_s=args.settle)
    except LinkError as error:
        raise Problem(str(error))

    session = ListeningSession(arm, None, make_motion, stop_mode=args.stop_mode,
                               on_message=on_message, status_every_s=args.status_every)
    link = ESP32Link(transport, on_trigger=session.on_trigger,
                     heartbeat_timeout_s=args.heartbeat_timeout,
                     require_heartbeat=args.require_heartbeat, echo=args.echo)
    session.link = link

    deadline = time.monotonic() + args.run_for if args.run_for else None
    with link:
        try:
            session.run(deadline=deadline)
        except KeyboardInterrupt:
            print('\nInterrupted.')
            session.shutdown()
        except SafetyError as error:
            session.shutdown()
            raise Problem(str(error))
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
