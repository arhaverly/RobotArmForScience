# The ESP32 ↔ arm link

`robotic_testing/listen.py` puts the arm into a **listening state**: it wanders around a
small volume, continuously, until the ESP32 says something. The moment it does, the arm
stops. This document is the contract between the two, so that firmware written against it
works the first time.

```
   ESP32  ──USB serial──▶  listen.py  ──Ethernet──▶  xArm controller
          ◀──────────────
```

---

## 1. The one rule

> **Any byte from the ESP32 stops the arm, except `~`.**

The reader thread in `esp32_link.py` fires the stop on the *first byte it sees*, before the
rest of the line has arrived and long before anything has parsed it. Deciding what the
message meant happens afterwards, on another thread, with the arm already stationary.

Three consequences you have to design the firmware around:

* **Don't chatter.** While the host is in `LISTENING`, anything the ESP32 sends halts the
  arm. Send nothing but the heartbeat until you actually want it to stop.
* **Noise stops the arm.** A wrong baud rate, a floating RX line or a stray boot banner
  reads as a stop request. That is the correct direction to fail in, but it will look like
  a mystery halt, so sections 2 and 9 are worth reading before you blame the code.
* **`~` (0x7E) is the one exemption.** It is the heartbeat — one bare byte, no newline, no
  other meaning. Nothing else may be sent unframed.

---

## 2. The physical link

| | |
|---|---|
| Transport | USB CDC / USB-UART, 8 data bits, no parity, 1 stop bit |
| Baud | **115200** (`--baud` to change; ignored on native USB CDC, which runs at USB speed) |
| Flow control | none — no RTS/CTS, no XON/XOFF |
| Encoding | 7-bit ASCII |
| Line ending | `\n`. A `\r` is ignored, so `Serial.println()` is fine |

### Which port, which `Serial`

| Board | `Serial` is | Host sees |
|---|---|---|
| ESP32 DevKitC / WROOM (CP2102, CH340) | UART0, bridged to USB | `/dev/ttyUSB0`, `COM4` |
| ESP32-S2 / S3 / C3, *USB CDC On Boot: Enabled* | the native USB port | `/dev/ttyACM0`, `COM4` |
| ESP32-S2 / S3 / C3, *USB CDC On Boot: Disabled* | UART0 on the pins; use `USBSerial` for USB | as above |

On a classic ESP32, UART0 is also where the ROM bootloader prints its banner at every
reset, and where `Serial.setDebugOutput(true)` sends the WiFi log. Keep the protocol port
clean: call `Serial.setDebugOutput(false)` and print your own debugging somewhere else (a
second UART, or just an LED). The host discards the first second of input after opening
the port (`--settle`) precisely so a banner cannot be read as a stop request, but anything
that arrives later will be.

### Opening the port does not reset your board

On every common ESP32 board DTR and RTS are wired to EN and BOOT, so opening a serial port
normally reboots the chip. `SerialTransport` holds both lines low from before the port is
opened, so connecting is invisible to the firmware and the ESP32 keeps running across a
restart of `listen.py`.

### On Windows, and on WSL2

Run `listen.py` from **Windows Python** (3.9 or newer) against the COM port directly —
`--port COM7`; `--list-ports` will tell you which one. That is the simplest arrangement and
the one the lab machine uses; see [LISTEN.md](LISTEN.md).

If you specifically want to run inside WSL2, a USB port on the Windows host is not visible
there without forwarding it first:

```powershell
winget install usbipd            # once
usbipd list                      # find the BUSID of the ESP32
usbipd attach --wsl --busid 1-3  # per session
```

after which it appears as `/dev/ttyACM0` (native USB) or `/dev/ttyUSB0` (CP210x/CH340)
inside WSL. The arm itself is reached over Ethernet, which works either way.

---

## 3. Framing

```
<VERB> [<arg> [<arg> ...]] \n
```

* Case-insensitive verb; the host upper-cases it. Arguments are split on whitespace.
* Maximum **120 bytes** per line. Longer than that with no `\n` and the host drops the
  line and reports `GARBAGE` — nearly always a baud mismatch.
* A bare `\n` is a valid, content-free **STOP**. Nothing has to be spelled out for the arm
  to halt.
* The heartbeat `~` is *not* a line. It carries no newline and can appear between any two
  bytes of anything else.

---

## 4. The heartbeat

While the host is `LISTENING`, the ESP32 sends a single `~` every **250 ms**. If the host
goes **1.5 s** without hearing anything (`--heartbeat-timeout`) it stops the arm and
reports `LINKLOST`. An ESP32 that has crashed, or a cable that has fallen out, is an ESP32
that can no longer stop the arm, so the host treats silence as a stop request.

The watchdog is **only armed once the first `~` has arrived**. Firmware that does not
implement the heartbeat is simply unmonitored, not constantly halted — so you can bring the
link up in stages. Pass `--require-heartbeat` to demand it from the start.

Send heartbeats only while the host has told you it is `LISTENING`. There is nothing to
watch over while the arm is already stopped.

**In the other direction**, the host sends `STATE` and `POSE` every 5 seconds while
listening (`--status-every`). That is your keepalive: if the ESP32 hears nothing from the
host for ~12 s, assume the host has gone.

---

## 5. ESP32 → computer

Every one of these stops the arm, because every one of them is a byte. The verb decides
what happens *after* the stop.

| Message | What the host does after stopping |
|---|---|
| `STOP` | nothing more; stays stopped. Replies `ACK STOP`. `HALT` and `ESTOP` are aliases |
| *(bare `\n`)* | identical to `STOP` |
| `RESUME` | starts idling again, re-arms the trigger. Replies `ACK RESUME`. `GO` and `START` are aliases |
| `MSG <text>` | replies `ACK MSG`, then prints the text — or, with `--plan`, sends it to the VLA planner and runs the resulting plan |
| `HOME` | drives back to the home pose, then stays stopped. Refused (`NACK HOME <reason>`) if the gripper is holding something |
| `HELLO <version>` | replies `READY`, then starts idling. Send this once after boot |
| `PING` | replies `PONG`. Remember this still stops the arm — use `~` to check the link, not `PING` |
| `STATE` / `POSE` | replies `POSE x y z` and `STATE <name>` |
| `QUIT` | shuts the session down cleanly, leaving the arm where it is. `BYE` and `EXIT` are aliases |
| *anything else* | `NACK <verb> unknown-verb`, and the arm stays stopped |
| `~` | **does not stop the arm.** Heartbeat only |

## 6. Computer → ESP32

Nothing here affects the arm; it is all status. Ignore any line you don't recognise —
verbs may be added.

| Message | Meaning |
|---|---|
| `READY <proto> <arm>` | host is up. `<proto>` is `1`, `<arm>` is `xarm7` or `xarm6` |
| `STATE LISTENING` | the arm is moving and the trigger is armed. **Start heartbeating** |
| `STATE STOPPED` | the arm is still, waiting for `RESUME`. Stop heartbeating |
| `STOPPED <ms>` | a stop completed. `<ms>` is from your byte to the arm being still |
| `POSE <x> <y> <z>` | gripper position in mm, arm base frame |
| `ACK <verb>` | your message was understood and acted on |
| `NACK <verb> <reason>` | it was not. `unknown-verb`, `check-baud-rate`, or a safety reason |
| `PONG` | reply to `PING` |
| `BYE` | the host is shutting down |

---

## 7. State machine

```
              ┌────────── power on ──────────┐
              ▼                              │
        ┌───────────┐   HELLO / RESUME   ┌───┴───────┐
        │  STOPPED  │ ─────────────────▶ │ LISTENING │
        │ arm still │                    │ arm moves │
        │           │ ◀───────────────── │ heartbeat │
        └───────────┘   any byte but ~   └───────────┘
                        heartbeat lost
                        idle motion faulted
```

The ESP32 mirrors this from the `STATE` lines the host sends. Both transitions are
announced, so the firmware never has to guess:

* entering `LISTENING` → `STATE LISTENING`
* entering `STOPPED` → `STOPPED <ms>`, then `STATE STOPPED` on the next status tick

`RESUME` is refused with a `NACK` if the arm cannot safely idle from where it is — holding
an object, gripper closed on the sample holder, or parked against the edge of the safety
envelope. The host prints the reason.

---

## 8. What "instantly" actually means

The stop path is: your byte → USB → host reader thread → `xArm.emergency_stop()` → the
controller abandons the move in flight.

| Stage | Typical |
|---|---|
| USB full-speed frame interval | 1 ms |
| host scheduling + reader thread | < 1 ms |
| `set_state(4)` over Ethernet to the controller | 2–10 ms |
| **command issued** | **~5–15 ms after your byte** |
| servos decelerating to a stand | tens to a few hundred ms, depending on speed |

The number in `STOPPED <ms>` is measured from the byte landing in the host to the stop
command being accepted. It does not include the mechanical deceleration, which is a
property of the arm and its commanded speed, not of this link. Idle motion runs at the
arm config's free-form profile (half speed on the xArm7), which keeps that ramp short.

`--stop-mode soft` exists for comparison: it lets the step in flight finish instead of
aborting it, which is gentler on the hardware and takes up to a full step — several hundred
milliseconds. `STOPPED <ms>` then reports the real, longer figure. Default is `hard`.

> **This is not a safety-rated emergency stop.** It is software, over USB, over a network,
> and it will not save anyone if the host hangs or the controller stops answering. The
> xArm's physical E-stop is the real one. Do not let the existence of this link change
> where people are allowed to stand.

---

## 9. When things go wrong

| Situation | What the host does |
|---|---|
| Cable unplugged, ESP32 crashed | heartbeat watchdog fires → stop, `LINKLOST` printed. Reconnect and `RESUME` |
| ESP32 resets and re-announces | `HELLO` stops the arm, host replies `READY` and resumes |
| Garbage on the line | stops the arm, `GARBAGE` + `NACK check-baud-rate` |
| Serial port disappears | stop, `LINKLOST`; the session stays up so the arm stays still |
| `listen.py` killed | the arm is left wherever it is. It does **not** keep moving — nothing else is commanding it |
| Arm faults, or idle motion can't find a reachable waypoint | the wander thread ends, the host reports why and goes to `STOPPED` |
| Message arrives in the gap before the trigger is armed | it is still stopped before the message is acted on; the invariant holds either way |

After a hard stop the controller is latched in its stop state. `RESUME` clears it
(`reset_safety_state()`) before moving again — the firmware does not have to do anything
special.

---

## 10. Reference firmware

Complete and working. A momentary button on GPIO4 (to GND) stops the arm, and pressing it
again resumes.

```cpp
/*
 * esp32_arm_link.ino -- speaks to robotic_testing/listen.py
 *
 * The contract: while the host is LISTENING, send nothing but a '~' every 250 ms.
 * Any other byte stops the arm.
 */
#include <string.h>

static const uint32_t LINK_BAUD       = 115200;
static const uint8_t  TRIGGER_PIN     = 4;      // momentary button to GND
static const uint8_t  LED_PIN         = 2;      // on-board LED on most DevKitC boards
static const uint32_t HEARTBEAT_MS    = 250;    // host gives up at 1500 ms
static const uint32_t HOST_TIMEOUT_MS = 12000;  // host ticks every 5000 ms
static const uint32_t DEBOUNCE_MS     = 30;

enum HostState { HOST_UNKNOWN, HOST_LISTENING, HOST_STOPPED };

static HostState hostState      = HOST_UNKNOWN;
static uint32_t  lastBeatAt     = 0;
static uint32_t  lastHostAt     = 0;
static char      line[96];
static uint8_t   lineLen        = 0;
static bool      buttonDown     = false;
static uint32_t  buttonChangedAt = 0;

void sendLine(const char *text) {
  Serial.print(text);
  Serial.print('\n');
  Serial.flush();          // push it out now -- this is the byte that stops the arm
}

void setup() {
  Serial.begin(LINK_BAUD);
  Serial.setDebugOutput(false);     // nothing but the protocol goes down this port
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // the host discards the first second after opening the port, so that the ROM boot
  // banner is never read as a stop request. Stay quiet until it is listening.
  delay(1500);
  sendLine("HELLO esp32-arm-link/1.0");
  lastHostAt = millis();
}

void handleHostLine(const char *text) {
  lastHostAt = millis();
  if (!strcmp(text, "STATE LISTENING")) {
    hostState  = HOST_LISTENING;
    lastBeatAt = millis();
  } else if (!strncmp(text, "STATE ", 6) || !strncmp(text, "STOPPED ", 8) ||
             !strncmp(text, "READY ", 6)) {
    hostState = HOST_STOPPED;
  } else if (!strcmp(text, "BYE")) {
    hostState = HOST_UNKNOWN;
  }
  // POSE / ACK / NACK / PONG are informational; ignore anything unrecognised
}

void readHost() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
      continue;
    }
    line[lineLen] = '\0';
    lineLen = 0;
    handleHostLine(line);
  }
}

void sendHeartbeat() {
  if (hostState != HOST_LISTENING) return;   // nothing to watch over while it is stopped
  uint32_t now = millis();
  if (now - lastBeatAt < HEARTBEAT_MS) return;
  lastBeatAt = now;
  Serial.write('~');       // exactly one byte, no newline: the one thing that does not stop
  Serial.flush();
}

void readButton() {
  bool down = (digitalRead(TRIGGER_PIN) == LOW);
  uint32_t now = millis();
  if (down == buttonDown) return;
  if (now - buttonChangedAt < DEBOUNCE_MS) return;
  buttonChangedAt = now;
  buttonDown = down;
  if (!down) return;                         // act on the press, not the release

  if (hostState == HOST_LISTENING) sendLine("STOP");
  else                             sendLine("RESUME");
}

void updateLed() {
  uint32_t now = millis();
  if (now - lastHostAt > HOST_TIMEOUT_MS)    digitalWrite(LED_PIN, (now / 100) & 1);
  else if (hostState == HOST_LISTENING)      digitalWrite(LED_PIN, (now / 500) & 1);
  else                                       digitalWrite(LED_PIN, HIGH);
}

void loop() {
  readHost();
  sendHeartbeat();
  readButton();
  updateLed();
}
```

To send an instruction rather than a bare stop, swap the `sendLine("STOP")` for something
like `sendLine("MSG pick up the blue cap")` and start the host with `--plan`. The arm stops
first, then the planner decides what to do about it.

---

## 11. Bringing it up

Work through this in order; each step fails in a way you can tell apart.

1. **The arm alone.** `python robotic_testing/listen.py --simulate --port stdin`. Press
   Enter — it should stop. Type `RESUME` — it should start again.
2. **The real arm, no ESP32.** Same command without `--simulate`. Watch it wander. Confirm
   the box it prints is somewhere you are happy for it to be, and that the physical E-stop
   is within reach.
3. **The ESP32, no arm.** Flash the sketch, open the Arduino serial monitor at 115200, and
   confirm you see `HELLO` and nothing else. If you see `~~~~` the firmware has the wrong
   idea about the host state. **Close the serial monitor before step 4** — two programs
   cannot hold the same port.
4. **Both, simulated arm.** `python robotic_testing/listen.py --simulate --port COM7
   --echo` (`--list-ports` finds the port; on Linux it is `/dev/ttyUSB0` or
   `/dev/ttyACM0`). Press the button; look for `*** STOP`.
5. **Both, real arm.** Drop `--simulate`. Start with `--radius 60` so the first run stays
   small, and widen it once you trust it.

Useful flags: `--echo` prints every line the ESP32 sends, `--for 30` quits after 30
seconds, `--stop-mode soft` for the gentler stop, `--no-wiggle` to keep the wrist still.

### Testing without an ESP32

```
python robotic_testing/check_esp32_link.py
```

A pty pair stands in for the board and the simulated arm stands in for the robot, so the
real reader thread, the real trigger rule and the real state machine all run. Each check
is one sentence from this document -- that `~` never stops the arm, that a bare newline
does, that a lost heartbeat does, that `STOPPED` reaches the ESP32 before the `ACK`, that
the wander box does not drift across resumes. If a check fails, this document and the code
disagree.

It needs a Unix pty, so on Windows it prints an explanation and exits; test by hand with
`--port stdin` there instead, or run the checks under WSL.

---

## 12. Changing the protocol

The verbs live in `ListeningSession.handle()` in `robotic_testing/listen.py`; the framing
and the trigger rule live in `robotic_testing/common/esp32_link.py`. If you add a verb,
bump `PROTOCOL_VERSION` in `esp32_link.py` — it is what the host announces in `READY`, so
firmware can tell which side is out of date.
