# Running the listener

`robotic_testing/listen.py` puts the arm into a **listening state**: it wanders slowly
around a small box, continuously, until the ESP32 sends something. The moment it does, the
arm stops — the stop is issued on the serial reader thread, before the message has even
finished arriving, let alone been parsed.

It keeps wandering until something asks it to stop. A waypoint the arm turns out not to be
able to reach is not commanded and another one is picked; a controller that faults is
cleared and idling starts again; a cable pulled out and pushed back in resumes on its own.
None of those is somebody asking for a stop, so none of them ends the session. See
[§7 below](#7-when-it-doesnt-work) for what that looks like, and `--no-auto-resume` if you
want the older behaviour where anything going wrong left the arm stopped until a human
sent `RESUME`.

This document is about *running* it, on the Windows 10 lab machine. The wire contract the
firmware has to speak — the verbs, the heartbeat, the framing, a complete reference sketch
— is [`ESP32_PROTOCOL.md`](ESP32_PROTOCOL.md).

Tested on **Python 3.9** and 3.14, on Windows and Linux. Nothing here needs WSL.

> **The one rule:** any byte from the ESP32 stops the arm, except `~` (the heartbeat).
> Noise, a wrong baud rate and a boot banner all stop the arm too. That is the safe
> direction to fail in, but it means a mystery halt is usually a wiring or baud problem.

---

## 1. Install (PowerShell)

```powershell
cd C:\path\to\RobotArmForScience
py -3.9 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r robotic_testing\requirements-vla.txt
```

If `Activate.ps1` is blocked by execution policy:

```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

You don't have to activate at all if you'd rather not — `.\.venv\Scripts\python.exe` works
directly anywhere below in place of `python`.

What each piece is actually for:

| You want to | You need |
|---|---|
| Try the listener from the keyboard (`--simulate`) | `numpy` only — no serial, no robot |
| Talk to a real ESP32 | `pyserial` |
| Drive the real arm | `xarm-python-sdk`, and the arm reachable at `192.168.1.209` over Ethernet |
| `--plan` (route `MSG` to the VLA planner) | `google-genai`, `python-dotenv`, and `GEMINI_API_KEY` in `.env` at the repo root |

`listen.py` puts the repo root on `sys.path` itself, so it runs from any directory.

---

## 2. The commands

```powershell
# Which COM port is the ESP32 on?
python robotic_testing\listen.py --list-ports

# No robot, no ESP32. The keyboard stands in for the board: press Enter to stop.
python robotic_testing\listen.py --simulate --port stdin

# Real ESP32, simulated arm. The safest way to test firmware.
python robotic_testing\listen.py --simulate --port COM7 --echo

# The real thing.
python robotic_testing\listen.py --port COM7

# The real thing, first run: a small box and a 30-second time limit.
python robotic_testing\listen.py --port COM7 --radius 60 --for 30

# Hand "MSG <text>" from the ESP32 to the VLA planner and run the resulting plan.
python robotic_testing\listen.py --port COM7 --plan
```

Substitute your own port for `COM7`. Ports above COM9 need no special spelling — pass
`--port COM12` and pyserial handles the `\\.\` prefix itself.

Bring a new setup up one stage at a time — arm alone, then ESP32 alone, then both — so each
failure is distinguishable. That ladder is [§11 of the protocol doc](ESP32_PROTOCOL.md).

### Finding the port

```powershell
python robotic_testing\listen.py --list-ports
```

```
USB serial ports -- the ESP32 is one of these:
  COM3  USB-Enhanced-SERIAL CH343  [1a86:7523]
  COM7  USB Serial Device  [303a:1001]

Look for Espressif, CP210x, CH340/CH343 or "USB Serial". A native-USB
S2/S3/C3 appears as a JTAG/serial debug unit, VID 303a.

(1 non-USB port(s), not the ESP32: COM1)
```

VID `303a` is Espressif, so `COM7` above is an ESP32-S2/S3/C3 on its native USB port.
`1a86` is WCH (CH340/CH343) and `10c4` is Silicon Labs (CP210x) — both are the USB-UART
bridge on a classic DevKitC. Cross-check against Device Manager → **Ports (COM & LPT)**.

**Close the Arduino IDE serial monitor before running the listener.** Two programs cannot
hold the same COM port, and this is the single most common cause of `could not open COM7`.

---

## 3. Trying it from the keyboard

`--port stdin` is the whole protocol with your keyboard as the ESP32. Type a verb and press
Enter; press Enter alone for a bare-newline stop. Lines prefixed `esp32 <-` are what the
host would be sending the board.

```
PS> python robotic_testing\listen.py --simulate --port stdin --echo
Simulated xarm7: no robot is connected and nothing will physically move.

Link: stdin (type a message and press Enter)
  esp32 <- READY 1 xarm7

LISTENING -- the arm is moving. Anything the ESP32 sends stops it.
Idling inside:
  x: 150.0 .. 450.0 mm
  y: 150.0 .. 450.0 mm
  z: 175.0 .. 325.0 mm
  steps of up to 40 mm, wrist joint 7 twisting up to 20 deg
  esp32 <- STATE LISTENING
  listening... 5 moves, at x=188 y=208 z=277 mm, 0 heartbeats
                                          <-- pressed Enter here
*** STOP -- a message arrived over USB
    the controller was halted 0.04 ms after the trigger
STOPPED at x=189.7 y=213.3 z=274.6 mm. Send RESUME to carry on, or press Ctrl-C to quit.
  esp32 <- STOPPED 0.04
  esp32 <- ACK STOP
RESUME                                    <-- typed
LISTENING -- the arm is moving. Anything the ESP32 sends stops it.
```

Worth knowing while you play with it:

* **Every verb stops the arm first.** `POSE`, `PING`, even a typo — they are all bytes. The
  verb only decides what happens *after* the stop. `~` is the sole exemption.
* **`RESUME` is what starts it moving again**, and it is what re-arms the trigger. It is
  only needed after a stop somebody asked for; the host retries the ones nobody asked for.
* **The wander box never drifts.** It is anchored at where the arm was when the process
  started, not where it was last halted, so a long shift of stops and resumes cannot walk
  the arm across the bench. Restart the process and the box re-anchors.
* `QUIT` (or Ctrl-C) leaves the arm exactly where it is, still.

The verbs in full — `STOP`, `RESUME`, `MSG`, `HOME`, `HELLO`, `PING`, `STATE`/`POSE`,
`QUIT` — are tabulated in [§5 of the protocol doc](ESP32_PROTOCOL.md).

---

## 4. The self-test

```powershell
python robotic_testing\check_esp32_link.py
```

On **Windows this cannot run** — it fakes a serial port with a Unix pty, and Windows has
none. It says so and exits rather than throwing a traceback. Test by hand with
`--simulate --port stdin` instead, or run it under WSL, where it passes 12/12:

```
PASS  six seconds of ~ heartbeats never stops the arm
PASS  and the arm keeps moving throughout
PASS  a message stops the arm
PASS  and STOPPED reaches the ESP32 before ACK
PASS  a bare newline stops the arm
PASS  a lost heartbeat stops the arm
PASS  a link that comes back starts the arm idling again with no RESUME
PASS  a stop somebody asked for is not undone by the automatic restart
PASS  an over-long line stops the arm
PASS  and is reported as a likely baud mismatch
PASS  the wander box is the same after every resume
PASS  the trigger is re-armed by RESUME, so the next message stops it too

12/12 checks passed
```

Each check is one sentence from the protocol document; a failure means the document and the
code disagree. Run it after touching `listen.py` or `esp32_link.py`.

---

## 5. Flags

**Link**

| Flag | Default | |
|---|---|---|
| `--list-ports` | | print the serial ports this machine can see, then exit |
| `--port` | `stdin` | COM port, or the literal `stdin` for the keyboard |
| `--baud` | `115200` | must match the firmware; a mismatch shows up as `GARBAGE` |
| `--settle` | `1.0` | seconds of input discarded after opening, so a boot banner is not read as a stop |
| `--heartbeat-timeout` | `1.5` | stop if the ESP32 goes this long without a `~`, once it has sent one |
| `--require-heartbeat` | off | demand the heartbeat from the start, rather than only after the first one |
| `--echo` | off | print every line the ESP32 sends. Use this while bringing firmware up |

**Arm**

| Flag | Default | |
|---|---|---|
| `--simulate` | off | no robot; nothing physically moves |
| `--arm` | `xarm7` | `xarm7` (270 platform) or `xarm6` (1318 platform) |
| `--stop-mode` | `hard` | `hard` aborts the move in flight in milliseconds; `soft` lets the step finish |

**Idle motion**

| Flag | Default | |
|---|---|---|
| `--radius` | `150.0` | half-width of the wander box, mm. Start smaller than this on a new setup |
| `--step` | `40.0` | length of one idle move, mm |
| `--no-wiggle` | off | keep the wrist still |
| `--no-ik-check` | off | stop asking the controller's own kinematics about a waypoint before walking to it. The envelope and path checks still apply. Only needed if a controller refuses poses it will then happily move to |
| `--speed-profile` | arm config | a `pos_settings_dict` key, e.g. `slow_2` |
| `--seed` | random | fix the wander for a repeatable demo |

**Carrying on by itself**

| Flag | Default | |
|---|---|---|
| `--no-auto-resume` | off | leave the arm stopped after anything goes wrong, instead of clearing what can be cleared and idling again. The ESP32 and the operator can stop it either way |
| `--max-fault-clears` | `10` | how many times a faulted controller may be cleared automatically before the session leaves it for a human to look at |

**Session**

| Flag | Default | |
|---|---|---|
| `--for SECONDS` | none | quit after this long. Good for a first trial run |
| `--status-every` | `5.0` | status line, and `STATE`/`POSE` to the ESP32, this often; `0` disables both |
| `--plan` | off | send `MSG <text>` to the VLA planner and run the plan it returns |
| `--model` | `gemini-3.5-flash` | planner model, with `--plan` |

The wander is clamped to the safety envelope the arm's config declares
(`common\robotic_arms\xarm7\xarm7_config.py`), so `--radius` widens the box only up to that
envelope — it can never talk the arm outside it.

---

## 6. Reading the output

| Line | Meaning |
|---|---|
| `LISTENING -- the arm is moving.` | trigger armed. The printed box is where it will wander |
| `listening... 12 moves, at x=… , 40 heartbeats` | the periodic tick. A rising heartbeat count means the link is live |
| `*** STOP -- a message arrived over USB` | the ESP32 spoke; the arm was already halted before this printed |
| `the controller was halted 0.04 ms after the trigger` | byte → stop command accepted. Not the mechanical deceleration |
| `STOPPED at x=… Send RESUME to carry on` | waiting. Nothing will move until `RESUME` |
| `The link to the ESP32 went quiet` | heartbeat watchdog fired. Check the cable; it starts idling again by itself once the board speaks |
| `The arm is still. Trying to idle again in 5 s` | something went wrong that nobody asked for. The arm is stopped meanwhile, and every precondition is re-checked before it moves |
| `(idling again by itself, after 2 attempt(s))` | it recovered. Nothing was skipped to get there |
| `listening... 40 moves, …, 3 candidate(s) skipped (3 on path)` | a waypoint was turned down while choosing, before anything moved. Normal — the wander box is a box and what the arm can reach is not. A steadily climbing count means a smaller `--radius` |
| `…, 2 waypoint(s) abandoned` | the arm was already walking there when the next step was refused, so it went somewhere else instead. Visible as a pause. Rare; if it is not, the envelope and the arm disagree about something |
| `note: only 12% of that box can actually be reached` | the box barely overlaps what the arm can reach from there. It will still idle, but hesitantly |
| `A line arrived with no newline in sight` | almost always a baud mismatch |
| `esp32 <- …` / `esp32 -> …` | host to board, and (with `--echo`) board to host |

`STOPPED <ms>` measures from the byte landing to the stop command being accepted by the
controller — typically 5–15 ms end to end. The servos then take tens to a few hundred
milliseconds to come to rest, which is a property of the arm and its commanded speed, not
of this link. See [§8 of the protocol doc](ESP32_PROTOCOL.md).

---

## 7. When it doesn't work

| Symptom | Cause |
|---|---|
| `could not open COM7` | the Arduino serial monitor still has it, or the wrong port. Run `--list-ports` |
| `pyserial is not installed` | `python -m pip install pyserial`, or use `--port stdin` |
| `Activate.ps1 cannot be loaded` | execution policy — see §1, or just call `.venv\Scripts\python.exe` directly |
| The arm stops the instant it starts | the ESP32 is chattering. While the host is `LISTENING` the firmware must send nothing but `~` |
| `GARBAGE` / `NACK check-baud-rate` | baud mismatch, or `Serial.setDebugOutput(true)` is spraying the WiFi log down the protocol port |
| Stops every second or so, `LINKLOST` | heartbeats are not arriving. Raise `--heartbeat-timeout`, or check the firmware is heartbeating while `LISTENING` |
| `Cannot idle: …` on `RESUME` | the arm cannot safely idle from where it is — holding something, or parked at the envelope edge. Send `HOME` first. It keeps retrying on its own, so a transient one needs nothing from you |
| The arm pauses a lot, `waypoint(s) not usable` climbing | the box overlaps the keep-out around the base column, or reaches past what the arm can actually get to with that tool orientation. Use a smaller `--radius`, or start it further from the base |
| It keeps clearing faults and idling again, and you want it to stop | `--no-auto-resume`, or send `STOP` — an asked-for stop is never undone |
| `This session has already cleared 10 fault(s)` | the arm is faulting repeatedly. That is a real problem on the arm; the host has stopped papering over it |
| No `~` in the status line but nothing stops | the firmware never sent a first heartbeat, so the watchdog is unarmed by design. `--require-heartbeat` makes that an error instead |
| Nothing after `Connecting to the real xarm7...` | the arm is reached over **Ethernet**, not USB. Check you can ping `192.168.1.209` |
| `No module named 'xarm'` | `python -m pip install xarm-python-sdk`, or add `--simulate` |

---

## 8. Safety

This is **not** a safety-rated emergency stop. It is software, over USB, over a network,
and it will not help if the host hangs or the controller stops answering. The xArm's
physical E-stop is the real one, and it should be within reach of whoever is running this.
Do not let this link change where people are allowed to stand.

Before the first run with a real arm: check the box `listen.py` prints is somewhere you are
happy for the arm to move unattended, and start with a `--radius` smaller than the default.

The automatic restarts are worth understanding before you leave it running. The host will
clear a controller fault and start the arm moving again without being asked — that is the
point of it — so an arm that faulted because it hit something will try again. Every attempt
re-runs the full precondition check (not holding anything, inside the envelope, a reachable
waypoint and a clear path to it), and repeated faults stop being cleared after
`--max-fault-clears`, but neither of those is a substitute for the physical E-stop or for
watching it. If you do not want a robot that restarts itself, run with `--no-auto-resume`.

---

## 9. Running it from WSL instead

Only needed if you want the self-test, which requires a Unix pty. A USB port is not visible
inside WSL2 without forwarding it from Windows first:

```powershell
usbipd list                          # find the BUSID of the ESP32
usbipd attach --wsl --busid 1-3      # per session; does not survive unplugging
```

The board then appears as `/dev/ttyACM0` (native USB) or `/dev/ttyUSB0` (CP210x/CH340)
inside WSL, and you need to be in the `dialout` group. Running from Windows avoids all of
this, which is why it is the default above.

---

## 10. Where the code is

| | |
|---|---|
| `listen.py` | the state machine, the verbs (`ListeningSession.handle`), the CLI |
| `common\esp32_link.py` | the transports, the framing, the trigger rule, the heartbeat watchdog |
| `common\idle_motion.py` | the wander: waypoints, the box, the wrist wiggle |
| `check_esp32_link.py` | the self-test, one check per sentence of the protocol doc |
| `ESP32_PROTOCOL.md` | the firmware contract and a working reference sketch |

Adding a verb means editing `ListeningSession.handle()` and bumping `PROTOCOL_VERSION` in
`esp32_link.py`, so firmware can tell which side is out of date.
