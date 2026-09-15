"""
The serial link between the ESP32 and the computer that drives the arm.

The point of this link is latency: while the arm is idling in its listening state, the
ESP32 is the thing that can stop it, and the stop has to happen in milliseconds rather
than at the end of whatever move is in flight. Two decisions follow from that.

* **Any byte stops the arm.** The reader thread fires ``on_trigger`` on the first byte
  it sees, before it has any idea what the message says. Parsing the line, deciding what
  the message *meant*, and resuming all happen afterwards, on the main thread. Garbage on
  the wire therefore stops the arm, which is the safe way round.
* **One byte is exempt.** ``~`` (:data:`HEARTBEAT_BYTE`) is the ESP32 saying "still here".
  It is the only byte that does not trigger, which is what lets the host tell a live link
  apart from an unplugged one without the heartbeat itself halting the arm every second.

The wire format is newline-terminated ASCII, ``VERB [args...]``. See ESP32_PROTOCOL.md
for the full protocol, the message vocabulary and a worked ESP32 sketch.
"""
import queue
import sys
import threading
import time

PROTOCOL_VERSION = '1'

# The one byte that does not stop the arm. Everything else does: on a link whose whole
# job is to halt a moving robot, an unrecognised byte is a reason to stop, not to guess.
HEARTBEAT_BYTE = b'~'

# Longer than any legal line. A run of bytes with no newline is noise, not a message.
MAX_LINE_BYTES = 120


class LinkError(Exception):
    """The serial link could not be opened or has failed. Carries an operator-readable message."""


class Message:
    """One parsed line from the ESP32."""

    __slots__ = ('verb', 'args', 'text', 'raw', 'received_at')

    def __init__(self, verb, args, raw, received_at):
        self.verb = verb
        self.args = args
        self.text = ' '.join(args)
        self.raw = raw
        self.received_at = received_at

    def __repr__(self):
        return f'Message({self.raw!r})'


class Trigger:
    """The instant the arm should stop, and why."""

    __slots__ = ('at', 'reason')

    def __init__(self, at, reason):
        self.at = at
        self.reason = reason

    def __repr__(self):
        return f'Trigger({self.reason!r})'


def parse_line(raw, received_at=None):
    """Turn one received line into a :class:`Message`, or None if it was blank."""
    tokens = raw.split()
    if not tokens:
        return None
    return Message(tokens[0].upper(), tokens[1:], raw, received_at or time.monotonic())


def _port_hint(suggest_listing=True):
    """
    Platform-specific advice for finding the right port name.

    ``suggest_listing`` is off when this is printed *by* --list-ports, which would
    otherwise be told to run itself.
    """
    if sys.platform == 'win32':
        listing = '  List what Windows can see:  python listen.py --list-ports\n' if suggest_listing else ''
        return (
            listing +
            '  Check Device Manager > Ports (COM & LPT) for the board.\n'
            '  Close the Arduino IDE serial monitor first -- it holds the port open.\n'
            '  Ports above COM9 need no special spelling here: --port COM12 is fine.'
        )
    listing = '  List the ports:  python listen.py --list-ports\n' if suggest_listing else ''
    return (
        listing +
        '  Check the cable, and that the board shows up:  ls /dev/ttyUSB* /dev/ttyACM*\n'
        '  You may need to be in the dialout group: sudo usermod -aG dialout $USER,\n'
        '  then log out and back in.\n'
        '  Make sure no serial monitor or IDE has the port open.'
    )


def describe_ports():
    """A printable list of the serial ports this machine can see, for --list-ports."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return ('pyserial is not installed, so the serial ports cannot be listed.\n'
                '  Install it with:  pip install pyserial')

    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    # Windows lists every COM port it has ever seen and WSL invents 64 /dev/ttyS*, so the
    # USB ones are separated out: the ESP32 is always a USB device, and always has a VID.
    usb = [port for port in ports if port.vid is not None]
    other = [port for port in ports if port.vid is None]

    lines = []
    if usb:
        lines.append('USB serial ports -- the ESP32 is one of these:')
        for port in usb:
            ident = f'{port.vid:04x}:{port.pid:04x}' if port.pid is not None else f'{port.vid:04x}'
            lines.append(f'  {port.device}  {port.description or "no description"}  [{ident}]')
        lines.append('')
        lines.append('Look for Espressif, CP210x, CH340/CH343 or "USB Serial". A native-USB')
        lines.append('S2/S3/C3 appears as a JTAG/serial debug unit, VID 303a.')
    else:
        lines.append('No USB serial ports found, so the ESP32 does not appear to be connected.')
        lines.append(_port_hint(suggest_listing=False))

    if other:
        shown = ', '.join(port.device for port in other[:4])
        suffix = ', ...' if len(other) > 4 else ''
        lines.append('')
        lines.append(f'({len(other)} non-USB port(s), not the ESP32: {shown}{suffix})')
    return '\n'.join(lines)


# ----------------------------------------------------------------------
# transports
# ----------------------------------------------------------------------

class SerialTransport:
    """
    A USB serial port, opened without disturbing the ESP32.

    Opening a serial port normally asserts DTR and RTS, and on every common ESP32 board
    those two lines are wired to EN/BOOT: the board reboots the moment the host connects,
    spraying the ROM bootloader banner down the link. Here they are held low from before
    the port is opened, and whatever does arrive in the first ``settle_s`` is discarded,
    so a reset that happens anyway cannot be mistaken for a stop request.
    """

    def __init__(self, port, baud=115200, settle_s=1.0, read_timeout_s=0.05):
        try:
            import serial
        except ImportError:
            raise LinkError(
                'pyserial is not installed, so the ESP32 cannot be reached.\n'
                '  Install it with:  pip install pyserial\n'
                '  Or drive the link from the keyboard instead:  --port stdin'
            )

        self.port = port
        self.baud = baud
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baud
        self._serial.timeout = read_timeout_s
        self._serial.write_timeout = 1.0
        # set before open(): pyserial applies the stored state as it configures the port,
        # so the lines are never asserted at all
        self._serial.dtr = False
        self._serial.rts = False
        try:
            self._serial.open()
        except Exception as error:
            raise LinkError(
                f'could not open {port} at {baud} baud: {error}\n'
                f'  Check the board is plugged in and the port name is right.\n'
                + _port_hint()
            )

        if settle_s > 0:
            time.sleep(settle_s)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    @property
    def description(self):
        return f'{self.port} at {self.baud} baud'

    def read(self):
        """Block until at least one byte arrives or the read times out. b'' on timeout."""
        chunk = self._serial.read(1)
        waiting = self._serial.in_waiting
        if chunk and waiting:
            chunk += self._serial.read(waiting)
        return chunk

    def write(self, data):
        self._serial.write(data)
        self._serial.flush()

    def close(self):
        try:
            self._serial.close()
        except Exception:
            pass


class StdinTransport:
    """
    The keyboard standing in for the ESP32, for trying the listening state out.

    Typing anything and pressing Enter is a message; pressing Enter on its own is a bare
    newline, which still counts as a byte and so still stops the arm.
    """

    description = 'stdin (type a message and press Enter)'

    def __init__(self):
        self._stream = sys.stdin.buffer
        self._closed = False

    def read(self):
        if self._closed:
            time.sleep(0.05)
            return b''
        try:
            chunk = self._stream.read(1)
        except (ValueError, OSError):
            chunk = b''
        if not chunk:
            # EOF: stop spinning, but stay alive so the session can shut down tidily
            self._closed = True
        return chunk

    def write(self, data):
        sys.stdout.write('  esp32 <- ' + data.decode('ascii', 'replace').rstrip('\r\n') + '\n')
        sys.stdout.flush()

    def close(self):
        self._closed = True


def open_transport(port, baud=115200, settle_s=1.0):
    """Build the transport named by ``port``; the literal string ``stdin`` uses the keyboard."""
    if port in ('stdin', '-'):
        return StdinTransport()
    return SerialTransport(port, baud=baud, settle_s=settle_s)


# ----------------------------------------------------------------------
# the link
# ----------------------------------------------------------------------

class ESP32Link:
    """
    Reads the ESP32, fires ``on_trigger`` on the first byte, queues parsed messages.

    ``on_trigger`` runs on the reader thread, with nothing between it and the byte that
    caused it, so it must do only the one thing that cannot wait -- halting the arm. It
    fires at most once per arming: the rest of the line that follows the trigger byte
    cannot trigger again, and :meth:`arm_trigger` is what re-arms it once the arm is
    moving once more.
    """

    def __init__(self, transport, on_trigger=None, heartbeat_timeout_s=1.5,
                 require_heartbeat=False, echo=False):
        self.transport = transport
        self.on_trigger = on_trigger
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.echo = echo
        self.messages = queue.Queue()

        self._armed = False
        self._triggered = False
        self._lock = threading.Lock()
        self._closing = threading.Event()
        self._buffer = bytearray()
        self._overflowed = False
        self._threads = []

        # the watchdog stays quiet until the first heartbeat arrives, so an ESP32 that
        # does not send them yet is merely unmonitored rather than constantly halting
        self._require_heartbeat = bool(require_heartbeat)
        self._heartbeat_seen = bool(require_heartbeat)
        self._last_heartbeat = time.monotonic()

        self.heartbeats = 0
        self.bytes_read = 0

    # --- lifecycle ----------------------------------------------------

    def start(self):
        self._threads = [
            threading.Thread(target=self._read_loop, name='esp32-reader', daemon=True),
            threading.Thread(target=self._watchdog_loop, name='esp32-watchdog', daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        return self

    def close(self):
        self._closing.set()
        self.transport.close()
        for thread in self._threads:
            thread.join(timeout=1.0)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc_info):
        self.close()

    # --- arming -------------------------------------------------------

    def arm_trigger(self):
        """Allow the next byte to fire ``on_trigger``. Call this whenever the arm starts moving."""
        with self._lock:
            self._armed = True
            self._triggered = False
            self._last_heartbeat = time.monotonic()
            # after a stop caused by a dead link the watchdog stays quiet until the ESP32
            # speaks again, so resuming does not immediately halt the arm a second time
            self._heartbeat_seen = self._require_heartbeat

    def disarm_trigger(self):
        """Stop bytes from firing ``on_trigger``; messages are still parsed and queued."""
        with self._lock:
            self._armed = False

    @property
    def armed(self):
        with self._lock:
            return self._armed

    # --- sending ------------------------------------------------------

    def send(self, verb, *args):
        """Send one line to the ESP32. Never touches the arm, so it is always safe to call."""
        line = ' '.join([str(verb)] + [str(arg) for arg in args])
        try:
            self.transport.write((line + '\n').encode('ascii', 'replace'))
        except Exception as error:
            # a dead uplink must not take the session down: the arm still has to be stopped
            print(f'  (could not send {line!r} to the ESP32: {error})')
            return False
        return True

    # --- reading ------------------------------------------------------

    def _fire(self, reason, at):
        """Stop the arm, once, as soon as possible. Runs on the reader/watchdog thread."""
        with self._lock:
            if self._triggered or not self._armed:
                return
            self._triggered = True
        if self.on_trigger is not None:
            try:
                self.on_trigger(Trigger(at, reason))
            except Exception as error:
                print(f'\nThe stop callback itself failed: {error}')

    def _read_loop(self):
        while not self._closing.is_set():
            try:
                chunk = self.transport.read()
            except Exception as error:
                if not self._closing.is_set():
                    self._fire(f'the serial link failed ({error})', time.monotonic())
                    self.messages.put(Message('LINKLOST', [str(error)], str(error), time.monotonic()))
                return
            if not chunk:
                continue

            at = time.monotonic()
            self.bytes_read += len(chunk)

            # the trigger decision is taken on the raw bytes, before any parsing: a line
            # that is still arriving must not delay the stop by the time it takes to finish
            if any(byte != HEARTBEAT_BYTE[0] for byte in chunk):
                self._fire('a message arrived over USB', at)

            for byte in chunk:
                self._consume(byte, at)

    def _consume(self, byte, at):
        if byte == HEARTBEAT_BYTE[0]:
            self.heartbeats += 1
            self._heartbeat_seen = True
            self._last_heartbeat = at
            return
        if byte == 0x0d:
            # lines are terminated by \n alone; a \r from Serial.println() is ignored, so
            # that a \r\n pair cannot be mistaken for two messages
            self._last_heartbeat = at
            return
        if byte == 0x0a:
            raw = self._buffer.decode('ascii', 'replace').strip()
            self._buffer.clear()
            if self._overflowed:
                self._overflowed = False
                self.messages.put(Message('GARBAGE', [], raw[:MAX_LINE_BYTES], at))
                return
            message = parse_line(raw, at)
            if message is None:
                # a bare newline is a deliberate, content-free stop: report it as one
                message = Message('STOP', [], '', at)
            if self.echo:
                print(f'  esp32 -> {message.raw or "<newline>"}')
            self.messages.put(message)
            return

        # any byte also counts as the link being alive, heartbeat or not
        self._last_heartbeat = at
        if len(self._buffer) >= MAX_LINE_BYTES:
            self._overflowed = True
            self._buffer.clear()
            return
        self._buffer.append(byte)

    def _watchdog_loop(self):
        """Treat a silent link as a stop: an ESP32 that cannot speak cannot stop the arm."""
        while not self._closing.wait(0.05):
            if not self._heartbeat_seen or not self.armed:
                continue
            silent_for = time.monotonic() - self._last_heartbeat
            if silent_for > self.heartbeat_timeout_s:
                at = time.monotonic()
                self._fire(f'no heartbeat from the ESP32 for {silent_for:.2f} s', at)
                self.messages.put(
                    Message('LINKLOST', [f'{silent_for:.2f}'], f'silent for {silent_for:.2f} s', at)
                )
                # do not re-report every 50 ms while the link stays down
                self._heartbeat_seen = False
