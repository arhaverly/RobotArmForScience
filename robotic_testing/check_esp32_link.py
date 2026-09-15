#!/usr/bin/env python3
"""
Check the ESP32 link does what ESP32_PROTOCOL.md promises, with no ESP32 and no robot.

A pty pair stands in for the board and the simulated arm stands in for the robot, so this
exercises the real reader thread, the real trigger rule and the real state machine:

    python check_esp32_link.py

Every check is one sentence from the protocol document. If one of them fails, the document
and the code disagree, and one of the two is wrong.

Needs a Unix pty, so it does not run on Windows. Test the link there by hand with
``python listen.py --simulate --port stdin``.
"""
import os
import pty
import re
import subprocess
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LISTEN = os.path.join(REPO_ROOT, 'robotic_testing', 'listen.py')

# long enough for the 1 s serial settle plus the first idle move to be under way
BOOT_S = 2.0


def run(script, extra_args=()):
    """Run listen.py against a pty, drive it with ``script``, return (its output, what it sent)."""
    master, slave = pty.openpty()
    proc = subprocess.Popen(
        [sys.executable, '-u', LISTEN, '--simulate', '--port', os.ttyname(slave),
         '--seed', '11', *extra_args],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, cwd=REPO_ROOT)
    printed, sent = [], []
    threading.Thread(target=lambda: printed.extend(proc.stdout), daemon=True).start()
    threading.Thread(target=_collect, args=(master, sent), daemon=True).start()
    try:
        script(master)
    finally:
        time.sleep(0.6)
        proc.terminate()
        proc.wait(timeout=5)
    return ''.join(printed), sent


def _collect(master, sent):
    buffer = b''
    while True:
        try:
            buffer += os.read(master, 512)
        except OSError:
            return
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            sent.append(line.decode('ascii', 'replace').strip())


def beat(master, seconds, period=0.25):
    deadline = time.time() + seconds
    while time.time() < deadline:
        os.write(master, b'~')
        time.sleep(period)


class Checks:
    def __init__(self):
        self.failures = 0
        self.total = 0

    def __call__(self, claim, ok, detail=''):
        self.total += 1
        if not ok:
            self.failures += 1
        print(f'{"PASS" if ok else "FAIL"}  {claim}')
        if not ok and detail:
            print(f'        {detail}')


def main():
    check = Checks()

    # "~ is the one exemption."
    def heartbeats_only(master):
        time.sleep(BOOT_S)
        beat(master, 6.0)
    printed, _ = run(heartbeats_only, ['--heartbeat-timeout', '1.0'])
    check('six seconds of ~ heartbeats never stops the arm', '*** STOP' not in printed)
    check('and the arm keeps moving throughout', 'listening...' in printed, printed[-400:])

    # "Any byte from the ESP32 stops the arm" / the stop precedes the reply
    def one_message(master):
        time.sleep(BOOT_S)
        os.write(master, b'STOP\r\n')
        time.sleep(1.0)
    printed, sent = run(one_message)
    replies = [line for line in sent if line.startswith(('STOPPED ', 'ACK STOP'))]
    check('a message stops the arm', '*** STOP' in printed)
    check('and STOPPED reaches the ESP32 before ACK',
          len(replies) >= 2 and replies[0].startswith('STOPPED ') and replies[1] == 'ACK STOP',
          str(replies))

    # "A bare \n is a valid, content-free STOP."
    def bare_newline(master):
        time.sleep(BOOT_S)
        os.write(master, b'\n')
        time.sleep(1.0)
    printed, _ = run(bare_newline)
    check('a bare newline stops the arm', '*** STOP' in printed)

    # "the host treats silence as a stop request"
    def drop_heartbeat(master):
        time.sleep(BOOT_S)
        beat(master, 2.0)
        time.sleep(2.0)
    printed, _ = run(drop_heartbeat, ['--heartbeat-timeout', '1.0'])
    check('a lost heartbeat stops the arm', 'no heartbeat' in printed, printed[-400:])

    # "Longer than that with no \n and the host drops the line and reports GARBAGE"
    def over_long(master):
        time.sleep(BOOT_S)
        os.write(master, bytes(range(0x21, 0x7f)) * 2 + b'\n')
        time.sleep(1.0)
    printed, _ = run(over_long)
    check('an over-long line stops the arm', '*** STOP' in printed)
    check('and is reported as a likely baud mismatch', 'baud rate' in printed, printed[-400:])

    # "Pass the same origin every time you restart, or the box follows the arm"
    def resume_repeatedly(master):
        time.sleep(BOOT_S)
        for _ in range(3):
            os.write(master, b'STOP\r\n')
            time.sleep(0.8)
            os.write(master, b'RESUME\r\n')
            time.sleep(1.2)
    printed, _ = run(resume_repeatedly)
    boxes = set(re.findall(r'x: (-?[\d.]+) \.\. (-?[\d.]+)', printed))
    check('the wander box is the same after every resume', len(boxes) == 1, str(boxes))

    # "RESUME ... starts idling again, re-arms the trigger"
    def stop_resume_stop(master):
        time.sleep(BOOT_S)
        os.write(master, b'STOP\r\n')
        time.sleep(0.8)
        os.write(master, b'RESUME\r\n')
        time.sleep(1.2)
        os.write(master, b'STOP\r\n')
        time.sleep(1.0)
    printed, _ = run(stop_resume_stop)
    check('the trigger is re-armed by RESUME, so the next message stops it too',
          printed.count('*** STOP') == 2, f'{printed.count("*** STOP")} stops')

    print(f'\n{check.total - check.failures}/{check.total} checks passed')
    return 1 if check.failures else 0


if __name__ == '__main__':
    sys.exit(main())
