import serial
from astropy.coordinates import SkyCoord, get_sun, EarthLocation, AltAz
from astropy.coordinates.name_resolve import NameResolveError
from astropy.time import Time
import astropy.units as u
from astroplan import Observer, FixedTarget
from parse import parse
from time import sleep
import logging

logging.basicConfig(level=logging.DEBUG)

class Caltech6m:
    
    Pasadena = EarthLocation.from_geodetic('-118d7.4650m', '34d8.3860m', '204.7m')
    obs = Observer(location=Pasadena, timezone='US/Pacific')
    
    def __init__(self, port='/dev/ttyUSB0'):
        logging.info('initializing telescope interface')
        self.serial = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.5,
        )
        logging.info('serial port opened')
        # TODO: better bookkeeping of _states. can't really trust them rn
        self._err_state = 'NONE'
        self._status = 'ALL_STOP'
        self.azatstow , self.elatstow, self.azmaint, self.elmaint = 179, 81, 270, 15
        self.azccwlim, self.azcwlim, self.eluplim, self.eldnlim = -89, 449, 81, 15
        self.startup()
        self.get_info()
        if input('calibrate telescope? [Y/n]: ') in ['Y', 'y', '']:
            self.calibrate()
        
    @property
    def currently_moving(self):
        self.get_info()
        if self.azerr > 1.0 or self.elerr > 1.0:
            sleep(1)
            self.get_info()
        return self.azerr > 1.0 or self.elerr > 1.0
    
    @property
    def error_state(self):
        return self._err_state
    
    @error_state.setter
    def error_state(self, value):
        self._err_state = value
        if value != 'NONE':
            logging.error('something weird happened: {}'.format(value))
            if input('continue? [Y/n]: ') not in ['Y', 'y', '']:
                self.spa()
                self.serial.close()
                raise Exception('user aborted')
    
    def startup(self):
        # clear out any junk in the serial buffer
        for i in range(8):
            self.serial.write(b'\r')
            sleep(.003)
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
        for cmd, pattern in seq:
            resp = self.send_command(cmd, send_delay=0.008, recv_delay=0.105)
            if pattern:
                if parse(pattern, resp) is None:
                    self.error_state = 'STARTUP_BAD_RESPONSE'
        logging.info('startup sequence complete')

    def send_command(self, cmd, send_delay=0.008, recv_delay=0.008):
        if isinstance(cmd, str): cmd = cmd.encode('utf-8')
        if cmd.endswith(b'\r') is False: cmd += b'\r'
        self.serial.write(cmd)
        logging.debug('Sent: {}'.format(cmd))
        sleep(send_delay)
        response = self.serial.read_until(b'\r')
        response = response.strip().decode('utf-8')
        logging.debug('Recv: {}'.format(response))
        sleep(recv_delay)
        if not response:
            self.error_state = 'NO_RESPONSE'
            return ''
        return response

    def brakes_on(self):
        self.get_info()
        if not self.AzBrkOn:
            self.send_command('ABN')
        if not self.ElBrkOn:
            self.send_command('EBN')
        self.get_info()
    
    def brakes_off(self):
        self.get_info()
        if self.AzBrkOn:
            self.send_command('ABF')
        if self.ElBrkOn:
            self.send_command('EBF')
        self.get_info()

    def point(self, az, el):
        self.get_info()
        if not self.calibrated:
            logging.error('cannot point telescope, it is not calibrated')
            return
        if self.currently_moving:
            logging.warning('telecope is currently moving, command ignored')
            return
        if el < self.eldnlim:
            logging.error('requested elevation is below horizon')
            return
        if el > self.eluplim:
            logging.error('requested elevation is above zenith')
            return
        if az < self.azccwlim:
            logging.error('requested azimuth is below azimuth limit')
            return
        if az > self.azcwlim:
            logging.error('requested azimuth is above azimuth limit')
            return
        if self.AzBrkOn:
            self.send_command('ABF')
        if self.ElBrkOn:
            self.send_command('EBF')
        return self.send_command('AZL,{:.3f},{:.3f}'.format(az, el))
    
    def point_radec(self, ra, dec):
        c = SkyCoord(ra, dec, unit='deg')
        altaz = c.transform_to(AltAz(obstime=Time.now(), location=self.Pasadena))
        return self.point(altaz.az.deg, altaz.alt.deg)
    
    def point_sun(self):
        t = Time.now()
        c = SkyCoord(get_sun(t), frame='gcrs', obstime=t, location=self.Pasadena)
        altaz = c.transform_to(AltAz(obstime=t, location=self.Pasadena))
        return self.point(altaz.az.deg, altaz.alt.deg)
    
    def point_object(self, name):
        t = Time.now()
        try:
            obj = FixedTarget.from_name(name)
        except NameResolveError:
            logging.error('could not resolve object name')
            return
        if not self.obs.target_is_up(t, obj, horizon=15*u.deg):
            logging.error('requested object is below horizon')
            return
        altaz = self.obs.altaz(t, obj)
        return self.point(altaz.az.deg, altaz.alt.deg)

    def stow(self):
        return self.point(self.azatstow, self.elatstow)
    
    def goto_maintenance_pos(self):
        return self.point(self.azmaint, self.elmaint)

    def get_info(self):
        sts = self.send_command('STS')
        sts = parse('STS,{mode:d}{ElUpPreLim:l}{ElDnPreLim:l}{ElUpFinLim:l}{ElDnFinLim:l}{AzCwPreLim:l}{AzCcwPreLim:l}{AzCwFinLim:l}{AzCcwFinLim:l}{AzLT180:l}{AzBrkOn:l}{ElBrkOn:l}{EmStopOn:l}{CalSts:d}{idk}', sts)
        if sts is not None:
            for k,v in sts.named.items():
                if isinstance(v, str):
                    if len(v) == 1:
                        setattr(self, k, (v == 'T'))
                if isinstance(v, int):
                    if k == 'mode':
                        self.mode = {0:'Stop', 1:'Calibrate', 5:'Track'}[v]
                    if k == 'CalSts':
                        self.CalSts = {0:'Not Calibrated', 1:'Calibrating Now', 2:'Calibration OK'}[v]

        azel = self.send_command('GAE')
        azel = parse('AZEL,{az:f},{el:f}', azel)
        if azel is not None:
            self.az = azel['az']
            self.el = azel['el']
            
        errs = self.send_command('ERR')
        errs = parse('ERR,{azerr:f},{elerr:f}', errs)
        if errs is not None:
            self.azerr = errs['azerr']
            self.elerr = errs['elerr']
            
        self.amp_currents = {
            '2A01': {
                'commanded': parse('2A01SIC{current:g}', self.send_command('2A01SIC'))['current'] or -999999,
                'actual': parse('2A01SIA{current:g}', self.send_command('2A01SIA'))['current'] or -999999,
            },
            '2A02': {
                'commanded': parse('2A02SIC{current:g}', self.send_command('2A02SIC'))['current'] or -999999,
                'actual': parse('2A02SIA{current:g}', self.send_command('2A02SIA'))['current'] or -999999,
            },
            '2A03': {
                'commanded': parse('2A03SIC{current:g}', self.send_command('2A03SIC'))['current'] or -999999,
                'actual': parse('2A03SIA{current:g}', self.send_command('2A03SIA'))['current'] or -999999,
            },
        }

    def status(self):
        self.get_info()
        if not self.calibrated:
            logging.warning('requested az and el values will not be correct, telescope is not calibrated')
        return self.az, self.el

    @property
    def calibrated(self):
        return self.CalSts == 'Calibration OK'
    
    def calibrate(self):
        if self.currently_moving:
            logging.warning('telecope is currently moving, command ignored')
            return
        
        seq = [
            ['SPA', 'SPA'],
            ['LPR,0.4,0.4,2.0,2.0,0.01,0.01,1.5,1.0,2.5,1.0,1.0,5.0,2.0,1.2,1.0,1.0,0.8,1.5,0.0,0.0,1.0,1.0,800,1200,-800,-1300,1.0,1.0,150,144000,144000,20000,20000,-301.087,118.5,1522.5,19749', 'LPR'],
            ['CLE', 'CLE'],
        ]
        self._status = 'CALIBRATING'
        logging.info('starting calibration')
        for cmd, pattern in seq:
            resp = self.send_command(cmd, send_delay=0.008, recv_delay=0.105)
            if pattern:
                if parse(pattern, resp) is None:
                    self.error_state = 'CAL_BAD_RESPONSE'
                    return
        self.get_info()
        while self.CalSts != 'Calibration OK':
            sleep(0.5)
            self.get_info()
        self._status = 'IDLE'
        logging.info('calibration complete')
        self.send_command('TMD,0')
        self.send_command('TON')
                
    def spa(self):
        self.send_command('SPA')
        self._status = 'ALL_STOP'