"""motors.py

Module for Controlling Different Motor Types over Serial

"""
import serial
from collections import deque
from enum import Enum
from queue import Empty, Queue
from threading import Event, Lock, Thread

from abc import ABC, abstractmethod
from time import sleep, monotonic
from math import cos, acos, pi, sqrt, floor

from astropy.coordinates import SkyCoord, get_sun, EarthLocation, AltAz
from astropy.coordinates.name_resolve import NameResolveError
from astropy.time import Time
import astropy.units as _u
from astroplan import Observer, FixedTarget
from parse import parse
import logging
from typing import Any, Dict


deg: Any = getattr(_u, "deg")


class Motor(ABC):
    """Abstract Class for All Motors Types

    Attributes
    ----------
    port : str
        Serial Port Identifier String for Communicating with the Motor
    baudrate : int
        Baudrate for serial connection
    az_limits : (float, float)
        Tuple of Lower and Upper Azimuth Limits
    el_limits : (float, float)
        Tuple of Lower and Upper Elevation Limits
    serial : serial.Serial
        Serial Object for Communicating with the Motor

    See Also
    --------
    <https://pyserial.readthedocs.io/en/latest/pyserial_api.html>
    """

    def __init__(self, port, baudrate, az_limits, el_limits):
        """Constructor for the Abstract Motor Class

        Parameters
        ----------
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        """
        self.port = port
        self.baudrate = baudrate
        self.az_limits = az_limits
        self.el_limits = el_limits
        self.serial = None
        self.serial_communications = deque(maxlen=200)

    def _record_serial_comm(self, direction, payload):
        """Record a serial communication event for dashboard diagnostics."""
        self.serial_communications.append({
            "time": Time.now().isot,
            "direction": direction,
            "payload": str(payload),
        })

    def get_recent_serial_communications(self, limit=12):
        """Returns the latest serial communication entries."""
        if limit <= 0:
            return []
        return list(self.serial_communications)[-limit:]

    @abstractmethod
    def point(self, az, el):
        """Abstract Method Prototype for Pointing a Motor at an AzEl Coordinate

        Parameters
        ----------
        az : float
            Azimuth Coordinate Value to Point At
        el : float
            Elevation Coordinate Value to Point At

        Returns
        -------
        (float, float)
            Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        pass

    @abstractmethod
    def status(self):
        """Abstract Method Prototype for Getting a Motor's Current AzEl Position

        Returns
        -------
        (float, float)
            Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        pass

    def __del__(self):
        """Override of Motor Delete Method to Close Serial Port if Necessary

        Returns
        -------
        None
        """
        if self.serial is not None and self.serial.is_open:
            self.serial.close()


class NoMotor(Motor):
    """
    Class for Simulating a Motor or Using a Stationary Telescope
    """

    def __init__(self, port, baudrate, az_limits, el_limits):
        """
        Initializer for Rot2Motor

        Parameters
        ----------
        port : str
            NOT USED - Needed For Abstract Motor Initializer
        baudrate : int
            Baudrate for serial connection
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits (if Stationary, both should be the same value)
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits (if Stationary, both should be the same value)
        """
        super().__init__(port, baudrate, az_limits, el_limits)
        self.position = (az_limits[0], el_limits[0])
        self.azerr = 0.0
        self.elerr = 0.0

    def point(self, az, el):
        """Changes the Unchanging Position of the Stationary / Simulated Motor

        Parameters
        ----------
        az : float
            Azimuth Coordinate to Point At
        el : float
            Elevation Coordinate to Point At

        Returns
        -------
        None
        """
        self._record_serial_comm("sent", f"SIM_POINT {az:.3f} {el:.3f}")
        self.position = (az, el)

    def status(self):
        """Returns the Unchanging Position of the Stationary / Simulated Motor

        Returns
        -------
        (float, float)
            Current Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        return self.position

class Caltech6m(Motor):

    READ_ONLY_COMMANDS = {
        "STS",
        "STX",
        "ERR",
        "GAE",
        "VER",
        "ENC",
        "NER",
        "2A01SIC",
        "2A01SIA",
        "2A02SIC",
        "2A02SIA",
        "2A03SIC",
        "2A03SIA",
    }

    ACK_PREFIX_BY_COMMAND = {
        "AZEL": "AZEL",
        "AZL": "AZL",
        "ZPN": "ZPN",
        "TON": "TON",
        "LPR": "LPR",
        "CLE": "CLE",
        "SPA": "SPA",
        "CLO": "CLO",
        "POF": "POF",
        "VOF": "VOF",
        "SEN": "SEN",
        "TMD": "TMD",
        "EBN": "EBN",
        "EBF": "EBF",
        "ABN": "ABN",
        "ABF": "ABF",
        "AZV": "AZV",
        "ELV": "ELV",
        "AVS": "AZS",
    }

    class State(Enum):
        DISCONNECTED = "disconnected"
        CONNECTING = "connecting"
        STARTUP_SYNC = "startup_sync"
        READY = "ready"
        SLEWING = "slewing"
        TRACKING = "tracking"
        CALIBRATING = "calibrating"
        FAULT = "fault"
        RECOVERING = "recovering"
        SHUTDOWN = "shutdown"
    
    Pasadena = EarthLocation.from_geodetic('-118d7.4650m', '34d8.3860m', 204.7)
    obs = Observer(location=Pasadena, timezone='US/Pacific')
    def __init__(self, port='COM1', baudrate=115200, az_limits=(-89,449), el_limits=(15,81), verbose=False, safe_mode=False):
        super().__init__(port, baudrate, az_limits, el_limits)
        logging.basicConfig(level=logging.INFO if verbose else logging.WARNING, format='%(asctime)s %(levelname)s: %(message)s')
        logging.warning('initializing telescope interface')
        self.safe_mode = bool(safe_mode)
        self._serial_lock = Lock()
        self._history_lock = Lock()
        self.state = Caltech6m.State.DISCONNECTED
        self.last_transition = Time.now().isot
        self.last_error = ''
        self.retry_count = 0
        self.max_command_retries = 2
        self.startup_retries = 2
        self.command_history = deque(maxlen=200)
        self._transition_state(Caltech6m.State.CONNECTING, 'opening serial port')
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.5,
        )
        self._command_queue = Queue()
        self._command_worker_stop = Event()
        self._command_worker = Thread(
            target=self._command_worker_loop,
            name="caltech6m-command-worker",
            daemon=True,
        )
        self._command_worker.start()
        # Default values set early so status/polling methods are safe before first full parse.
        self.mode = 'Stop'
        self.CalSts = 'Not Calibrated'
        self.AzBrkOn = False
        self.ElBrkOn = False
        self.az = float(az_limits[0])
        self.el = float(el_limits[0])
        self.azerr = 0.0
        self.elerr = 0.0
        self.amp_currents = {
            '2A01': {'commanded': -999999, 'actual': -999999},
            '2A02': {'commanded': -999999, 'actual': -999999},
            '2A03': {'commanded': -999999, 'actual': -999999},
        }
        logging.warning('serial port opened')
        try:
            if self.safe_mode:
                logging.warning('Caltech6m safe mode enabled: status-only (motion/control commands blocked)')
                self._transition_state(Caltech6m.State.READY, 'safe mode status-only startup')
            else:
                self.startup()
            self.get_info()
        except Exception:
            self._shutdown_command_worker()
            if self.serial is not None and self.serial.is_open:
                self.serial.close()
            raise
        # if not self.calibrated:
        #     if input('telescope is not calibrated, would you like to calibrate now? [y]/n') != 'n':
        #         self.calibrate()

    def _transition_state(self, new_state, reason=''):
        if self.state == new_state:
            return
        prev = self.state
        self.state = new_state
        self.last_transition = Time.now().isot
        if reason:
            logging.warning('state transition: %s -> %s (%s)', prev.value, new_state.value, reason)
        else:
            logging.warning('state transition: %s -> %s', prev.value, new_state.value)

    def _set_fault(self, message):
        self.last_error = message
        logging.error(message)
        self._transition_state(Caltech6m.State.FAULT, message)

    def _read_response_line(self):
        response = self.serial.read_until(b'\r')
        response = response.strip().decode('utf-8', errors='replace')
        if response:
            self._record_serial_comm("recv", response)
        return response

    def _command_text(self, cmd):
        if isinstance(cmd, bytes):
            return cmd.decode('utf-8', errors='replace').strip()
        return str(cmd).strip()

    def _verify_response(self, command_text, response, expected_prefix=None):
        if expected_prefix:
            return response.startswith(expected_prefix)
        if not response:
            return False
        token = command_text.split(',')[0].upper()
        ack_prefix = Caltech6m.ACK_PREFIX_BY_COMMAND.get(token)
        if ack_prefix is not None:
            return response.startswith(ack_prefix)
        return True

    def _read_matching_response(self, command_text, expected_prefix=None, max_reads=4, max_wait_s=0.25):
        """Read a few serial lines and return the first response that validates.

        Hardware can occasionally return a delayed response from a previous command,
        causing an apparent off-by-one reply stream. This helper tolerates that by
        scanning a bounded number of lines for a response that matches this command.
        """
        deadline = monotonic() + max(0.01, float(max_wait_s))
        reads = 0
        last_response = ''

        while reads < max(1, int(max_reads)) and monotonic() <= deadline:
            response = self._read_response_line()
            reads += 1
            if not response:
                continue

            last_response = response
            if self._verify_response(command_text, response, expected_prefix=expected_prefix):
                return response, True

            logging.warning(
                'out-of-sync response while waiting for %s: %r',
                command_text,
                response,
            )

        return last_response, False

    def _record_command_history(self, command_text, expected_prefix, response, retries, error, start_mono, end_mono, enqueued_mono):
        queue_wait_ms = max(0.0, (start_mono - enqueued_mono) * 1000.0)
        duration_ms = max(0.0, (end_mono - start_mono) * 1000.0)
        total_ms = max(0.0, (end_mono - enqueued_mono) * 1000.0)
        with self._history_lock:
            self.command_history.append(
                {
                    "time": Time.now().isot,
                    "command": command_text,
                    "expected": expected_prefix,
                    "response": str(response),
                    "retries": int(retries),
                    "success": (error == ""),
                    "error": str(error),
                    "queue_wait_ms": round(queue_wait_ms, 2),
                    "duration_ms": round(duration_ms, 2),
                    "total_ms": round(total_ms, 2),
                }
            )

    def get_recent_command_history(self, limit=12):
        if limit <= 0:
            return []
        with self._history_lock:
            return list(self.command_history)[-limit:]

    def _send_command_direct(
        self,
        cmd,
        command_text,
        send_delay,
        recv_delay,
        expected_prefix,
        retries,
        fail_on_error,
    ):
        last_response = ''
        for attempt in range(retries + 1):
            with self._serial_lock:
                self._record_serial_comm("sent", command_text)
                self.serial.write(cmd)
                logging.info('Sent: %s', cmd)
                sleep(send_delay)
                response, matched = self._read_matching_response(
                    command_text,
                    expected_prefix=expected_prefix,
                    max_reads=5,
                    max_wait_s=max(0.05, recv_delay + 0.05),
                )
                logging.info('Recv: %s', response)
                if not matched and not response:
                    self.serial.write(b'\r')
                    sleep(send_delay)
                    response, matched = self._read_matching_response(
                        command_text,
                        expected_prefix=expected_prefix,
                        max_reads=4,
                        max_wait_s=max(0.05, recv_delay + 0.05),
                    )
                    logging.info('Probe Recv: %s', response)
                sleep(recv_delay)

                if not matched:
                    # One more bounded read pass before resending command.
                    extra_response, extra_matched = self._read_matching_response(
                        command_text,
                        expected_prefix=expected_prefix,
                        max_reads=5,
                        max_wait_s=max(0.08, float(self.serial.timeout or 0.5)),
                    )
                    if extra_response:
                        response = extra_response
                    matched = extra_matched

            last_response = response
            if matched:
                return response, attempt, ''

            logging.warning(
                'command %s failed validation on attempt %d/%d, response=%r',
                command_text,
                attempt + 1,
                retries + 1,
                response,
            )

        err = f'command failed after retries: {command_text} response={last_response!r}'
        if fail_on_error:
            self._set_fault(err)
        else:
            logging.error(err)
        return '', retries + 1, err

    def _command_worker_loop(self):
        while not self._command_worker_stop.is_set():
            try:
                req = self._command_queue.get(timeout=0.1)
            except Empty:
                continue
            if req is None:
                self._command_queue.task_done()
                break
            req['start_mono'] = monotonic()
            response, retry_count, error = self._send_command_direct(
                req['cmd'],
                req['command_text'],
                req['send_delay'],
                req['recv_delay'],
                req['expected_prefix'],
                req['retries'],
                req['fail_on_error'],
            )
            req['response'] = response
            req['retry_count'] = retry_count
            req['error'] = error
            req['end_mono'] = monotonic()
            if not req.get('history_recorded', False):
                self._record_command_history(
                    command_text=req['command_text'],
                    expected_prefix=req['expected_prefix'],
                    response=response,
                    retries=retry_count,
                    error=error,
                    start_mono=req['start_mono'],
                    end_mono=req['end_mono'],
                    enqueued_mono=req['enqueued_mono'],
                )
                req['history_recorded'] = True
            req['done'].set()
            self._command_queue.task_done()

    def _shutdown_command_worker(self):
        if hasattr(self, '_command_worker_stop'):
            self._command_worker_stop.set()
        if hasattr(self, '_command_queue'):
            self._command_queue.put(None)
        if hasattr(self, '_command_worker') and self._command_worker.is_alive():
            self._command_worker.join(timeout=1.5)
    
    def startup(self):
        self._transition_state(Caltech6m.State.STARTUP_SYNC, 'running startup sequence')
        # clear out any junk in the serial buffer
        for i in range(8):
            self.serial.write(b'\r')
            sleep(.003)
        self.serial.read_until()
        seq = [
            ['2A01RST', ''],
            ['2A01RST', ''],
            ['2A02RST', ''],
            ['2A03RST', ''],
            ['2A01EN1', ''],
            ['2A02EN1', ''],
            ['2A03EN1', ''],
            ['VER', ''],
            ['VER', 'Ver 6mCBASS4'],
        ]
        startup_ok = False
        for attempt in range(self.startup_retries + 1):
            startup_ok = True
            for cmd, pattern in seq:
                expected = 'Ver' if pattern.startswith('Ver') else None
                resp = self.send_command(
                    cmd,
                    send_delay=0.008,
                    recv_delay=0.105,
                    expected_prefix=expected,
                    retries=self.max_command_retries,
                    fail_on_error=False,
                )
                if not resp:
                    startup_ok = False
                    break
                if pattern and parse(pattern, resp) is None:
                    startup_ok = False
                    break
            if startup_ok:
                break
            self._transition_state(
                Caltech6m.State.RECOVERING,
                f'startup retry {attempt + 1}/{self.startup_retries + 1}',
            )
            sleep(0.1)
        if not startup_ok:
            self._set_fault('startup sequence failed')
            raise RuntimeError('Caltech6m startup sequence failed')
        self._transition_state(Caltech6m.State.READY, 'startup sequence complete')
        logging.warning('startup sequence complete')

    def send_command(
        self,
        cmd,
        send_delay=0.008,
        recv_delay=0.008,
        expected_prefix=None,
        retries=None,
        fail_on_error=True,
    ):
        if retries is None:
            retries = self.max_command_retries
        if isinstance(cmd, str):
            cmd = cmd.encode('utf-8')
        if not cmd.endswith(b'\r'):
            cmd += b'\r'
        command_text = self._command_text(cmd)
        command_token = command_text.split(',')[0].upper()

        if self.safe_mode and command_token not in Caltech6m.READ_ONLY_COMMANDS:
            logging.warning('safe mode blocked command: %s', command_text)
            return ''

        if self.state == Caltech6m.State.SHUTDOWN:
            logging.error('cannot send command while shutdown: %s', command_text)
            return ''

        serial_timeout = float(self.serial.timeout or 0.5)
        wait_timeout = (retries + 1) * (
            send_delay + recv_delay + 2 * serial_timeout + 0.2
        ) + 1.0

        req = {
            'cmd': cmd,
            'command_text': command_text,
            'send_delay': send_delay,
            'recv_delay': recv_delay,
            'expected_prefix': expected_prefix,
            'retries': retries,
            'fail_on_error': fail_on_error,
            'response': '',
            'retry_count': retries + 1,
            'error': '',
            'done': Event(),
            'enqueued_mono': monotonic(),
            'start_mono': 0.0,
            'end_mono': 0.0,
            'history_recorded': False,
        }
        self._command_queue.put(req)
        if not req['done'].wait(wait_timeout):
            err = f'command worker timed out: {command_text}'
            self.retry_count = retries + 1
            if fail_on_error:
                self._set_fault(err)
            else:
                logging.error(err)
            req['end_mono'] = monotonic()
            if not req['history_recorded']:
                start_mono = req['start_mono'] or req['enqueued_mono']
                self._record_command_history(
                    command_text=command_text,
                    expected_prefix=expected_prefix,
                    response='',
                    retries=retries + 1,
                    error=err,
                    start_mono=start_mono,
                    end_mono=req['end_mono'],
                    enqueued_mono=req['enqueued_mono'],
                )
                req['history_recorded'] = True
            return ''

        self.retry_count = req['retry_count']
        if not req['error']:
            self.last_error = ''
        return req['response']

    @staticmethod
    def _parse_named(pattern: str, text: str) -> Dict[str, Any]:
        """Parse helper that returns a plain dict of named fields."""
        parsed = parse(pattern, text)
        if parsed is None:
            return {}
        named = getattr(parsed, 'named', None)
        if isinstance(named, dict):
            return named
        return {}

    def brakes_on(self):
        self.get_info()
        if not self.AzBrkOn:
            self.send_command('ABN', expected_prefix='ABN')
        if not self.ElBrkOn:
            self.send_command('EBN', expected_prefix='EBN')
        self.get_info()
    
    def brakes_off(self):
        self.get_info()
        if self.AzBrkOn:
            self.send_command('ABF', expected_prefix='ABF')
        if self.ElBrkOn:
            self.send_command('EBF', expected_prefix='EBF')
        self.get_info()

    def point(self, az, el):
        if not self.calibrated:
            logging.error('cannot point telescope, it is not calibrated')
            return
        if az < self.az_limits[0] or az > self.az_limits[1]:
            logging.error('requested azimuth %.3f is outside limits [%s, %s]', az, self.az_limits[0], self.az_limits[1])
            return
        if el < self.el_limits[0]:
            logging.error('requested location is below horizon (min %.3f deg elevation)', self.el_limits[0])
            return
        if el > self.el_limits[1]:
            logging.error('requested object is higher than zenith (max %.3f deg elevation)', self.el_limits[1])
            return
        self.brakes_off()
        self.send_command('TON', expected_prefix='TON')
        self._transition_state(Caltech6m.State.SLEWING, f'point command to az={az:.3f}, el={el:.3f}')
        resp = self.send_command(f"AZL,{az},{el}", expected_prefix='AZL')
        self.get_info()
        return resp
    
    def point_radec(self, ra, dec):
        c = SkyCoord(ra, dec, unit='deg')
        altaz = c.transform_to(AltAz(obstime=Time.now(), location=self.Pasadena))
        if altaz.az is None or altaz.alt is None:
            logging.error('failed to transform RA/Dec to AltAz')
            return
        return self.point(altaz.az.deg, altaz.alt.deg)
    
    def point_sun(self):
        t = Time.now()
        c = SkyCoord(get_sun(t), frame='gcrs', obstime=t, location=self.Pasadena)
        altaz = c.transform_to(AltAz(obstime=t, location=self.Pasadena))
        if altaz.az is None or altaz.alt is None:
            logging.error('failed to transform Sun to AltAz')
            return
        return self.point(altaz.az.deg, altaz.alt.deg)
    
    def point_object(self, name):
        t = Time.now()
        try:
            obj = FixedTarget.from_name(name)
        except NameResolveError:
            logging.error('could not resolve object name')
            return
        if not self.obs.target_is_up(t, obj, horizon=15 * deg):
            logging.error('requested object is below horizon')
            return
        altaz = self.obs.altaz(t, obj)
        if altaz.az is None or altaz.alt is None:
            logging.error('failed to transform target object to AltAz')
            return
        return self.point(altaz.az.deg, altaz.alt.deg)
    
    def stow(self):
        self.brakes_off()
        if not self.calibrated:
            logging.warning('telescope is not calibrated, only moving elevation')
            return self.send_command('ELV,1200', expected_prefix='ELV')
        return self.point(179,81)
    
    def maint(self):
        self.brakes_off()
        if not self.calibrated:
            logging.warning('telescope is not calibrated, only moving elevation')
            return self.send_command('ELV,-1200', expected_prefix='ELV')
        return self.point(270,15)
    
    def cw(self, time=-1, speed=500):
        self.brakes_off()
        self.send_command(f"AZV,{abs(speed)}", expected_prefix='AZV')
        if time>0: 
            sleep(time)
            self.send_command('AZV,0', expected_prefix='AZV')
        
    def ccw(self, time=-1, speed=500):
        self.brakes_off()
        self.send_command(f"AZV,{-abs(speed)}", expected_prefix='AZV')
        if time>0: 
            sleep(time)
            self.send_command('AZV,0', expected_prefix='AZV')
                    
    def up(self, time=-1, speed=1100):
        self.brakes_off()
        self.send_command(f"ELV,{abs(speed)}", expected_prefix='ELV')
        if time>0: 
            sleep(time)
            self.send_command('ELV,0', expected_prefix='ELV')
    
    def down(self, time=-1, speed=1100):
        self.brakes_off()
        self.send_command(f"ELV,{-abs(speed)}", expected_prefix='ELV')
        if time>0: 
            sleep(time)
            self.send_command('ELV,0', expected_prefix='ELV')

    def get_info(self):
        # self.serial.write(b'\r')
        # self.serial.read_until(b'\r')
        sts = self.send_command('STS', expected_prefix='STS', fail_on_error=False)
        if not sts:
            return
        sts_named = self._parse_named(
            'STS,{mode:d}{ElUpPreLim:l}{ElDnPreLim:l}{ElUpFinLim:l}{ElDnFinLim:l}{AzCwPreLim:l}{AzCcwPreLim:l}{AzCwFinLim:l}{AzCcwFinLim:l}{AzLT180:l}{AzBrkOn:l}{ElBrkOn:l}{EmStopOn:l}{CalSts:d}{idk}',
            sts,
        )
        if sts_named:
            for k, v in sts_named.items():
                if isinstance(v, str):
                    if len(v) == 1:
                        setattr(self, k, (v == 'T'))
                if isinstance(v, int):
                    if k == 'mode':
                        self.mode = {0:'Stop', 1:'Calibrate', 5:'Track'}.get(v, f'Unknown({v})')
                    if k == 'CalSts':
                        self.CalSts = {0:'Not Calibrated', 1:'Calibrating Now', 2:'Calibration OK'}.get(v, f'Unknown({v})')
                logging.info('{}: {}'.format(k, v))

        if self.mode == 'Track':
            self._transition_state(Caltech6m.State.TRACKING, 'controller mode indicates tracking')
        elif self.mode == 'Calibrate' or self.CalSts == 'Calibrating Now':
            self._transition_state(Caltech6m.State.CALIBRATING, 'controller mode indicates calibration')
        elif self.state not in (Caltech6m.State.SHUTDOWN, Caltech6m.State.FAULT):
            self._transition_state(Caltech6m.State.READY, 'status refresh')

        azel = self.send_command('GAE', expected_prefix='AZEL', fail_on_error=False)
        azel_named = self._parse_named('AZEL,{az:f},{el:f}', azel)
        if azel_named:
            self.az = float(azel_named.get('az', self.az))
            self.el = float(azel_named.get('el', self.el))
            
        errs = self.send_command('ERR', expected_prefix='ERR', fail_on_error=False)
        errs_named = self._parse_named('ERR,{azerr:f},{elerr:f}', errs)
        if errs_named:
            self.azerr = float(errs_named.get('azerr', self.azerr))
            self.elerr = float(errs_named.get('elerr', self.elerr))
            
        sic1 = self._parse_named('2A01{current:d}', self.send_command('2A01SIC', expected_prefix='2A01', fail_on_error=False))
        sia1 = self._parse_named('2A01{current:d}', self.send_command('2A01SIA', expected_prefix='2A01', fail_on_error=False))
        sic2 = self._parse_named('2A02{current:d}', self.send_command('2A02SIC', expected_prefix='2A02', fail_on_error=False))
        sia2 = self._parse_named('2A02{current:d}', self.send_command('2A02SIA', expected_prefix='2A02', fail_on_error=False))
        sic3 = self._parse_named('2A03{current:d}', self.send_command('2A03SIC', expected_prefix='2A03', fail_on_error=False))
        sia3 = self._parse_named('2A03{current:d}', self.send_command('2A03SIA', expected_prefix='2A03', fail_on_error=False))
        self.amp_currents = {
            '2A01': {
                'commanded': int(sic1.get('current', -999999)),
                'actual': int(sia1.get('current', -999999)),
            },
            '2A02': {
                'commanded': int(sic2.get('current', -999999)),
                'actual': int(sia2.get('current', -999999)),
            },
            '2A03': {
                'commanded': int(sic3.get('current', -999999)),
                'actual': int(sia3.get('current', -999999)),
            },                    
        }
        # self.serial.write(b'\r')
        # self.serial.read_until(b'\r')

    def status(self):
        self.get_info()
        if not self.calibrated:
            logging.warning('requested az and el values will not be correct, telescope is not calibrated')
        return self.az, self.el

    @property
    def calibrated(self):
        return self.CalSts == 'Calibration OK'
    
    def calibrate(self):
        seq = [
            ['SPA', 'SPA'],
            ['LPR,0.4,0.4,2.0,2.0,0.01,0.01,1.5,1.0,2.5,1.0,1.0,5.0,2.0,1.2,1.0,1.0,0.8,1.5,0.0,0.0,1.0,1.0,800,1200,-800,-1300,1.0,1.0,150,144000,144000,20000,20000,-301.087,118.5,1522.5,19749', 'LPR'],
            ['CLE', 'CLE'],
        ]
        logging.warning('starting calibration')
        self._transition_state(Caltech6m.State.CALIBRATING, 'calibration started')
        for cmd, pattern in seq:
            resp = self.send_command(
                cmd,
                send_delay=0.008,
                recv_delay=0.105,
                expected_prefix=pattern,
                retries=self.max_command_retries,
            )
            if pattern:
                if parse(pattern, resp) is None:
                    logging.error('calibration bad response: {}'.format(resp))
                    self._set_fault('calibration command failed')
                    return
        self.get_info()
        start = monotonic()
        timeout_s = 180.0
        while self.CalSts != 'Calibration OK':
            if monotonic() - start > timeout_s:
                self._set_fault('calibration timed out')
                return
            sleep(0.5)
            self.get_info()
        logging.warning('calibration complete')
        self.send_command('TMD,0', expected_prefix='TMD')
        self.send_command('TON', expected_prefix='TON')
        self._transition_state(Caltech6m.State.READY, 'calibration complete')
                
    def spa(self):
        self.send_command('SPA', expected_prefix='SPA', fail_on_error=False)
        if self.state not in (Caltech6m.State.FAULT, Caltech6m.State.SHUTDOWN):
            self._transition_state(Caltech6m.State.READY, 'stop-all command')

    def stop(self):
        self.spa()

    def get_fsm_status(self):
        return {
            'state': self.state.value,
            'last_transition': self.last_transition,
            'retry_count': self.retry_count,
            'last_error': self.last_error,
            'safe_mode': self.safe_mode,
        }
        
    def cleanup(self):
        self._transition_state(Caltech6m.State.SHUTDOWN, 'cleanup invoked')
        self.spa()
        self.brakes_on()
        self._shutdown_command_worker()
        if self.serial is not None and self.serial.is_open:
            self.serial.close()