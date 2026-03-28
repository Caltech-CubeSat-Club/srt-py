import serial
from astropy.coordinates import SkyCoord, get_sun, EarthLocation, AltAz
from astropy.coordinates.name_resolve import NameResolveError
from astropy.time import Time
import astropy.units as u
from astroplan import Observer, FixedTarget
from parse import parse
from time import sleep
import logging
from astroquery.simbad import Simbad
import paho.mqtt.client as paho
from paho import mqtt

class Caltech6m:
    
    Pasadena = EarthLocation.from_geodetic('-118d7.4650m', '34d8.3860m', '204.7m')
    obs = Observer(location=Pasadena, timezone='US/Pacific')
    def __init__(self, port='COM1', baudrate=115200, az_limits=(-89,449), el_limits=(15,81), verbose=False, mqtt_broker='localhost', mqtt_port=1883, mqtt_username=None, mqtt_password=None):
        logging.basicConfig(level=logging.INFO if verbose else logging.WARNING, format='%(asctime)s %(levelname)s: %(message)s')
        logging.warning('initializing telescope interface')
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.5,
        )
        self.az_limits = az_limits
        self.el_limits = el_limits
        logging.warning('serial port opened')
        self.startup()
        self.get_info()
        if not self.calibrated:
            if input('telescope is not calibrated, would you like to calibrate now? [y]/n') != 'n':
                self.calibrate()
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client = paho.Client(client_id="telescope", userdata=None, protocol=paho.MQTTv5)
        self.mqtt_client.on_connect = self.on_connect
        # Enable TLS for secure connection
        self.mqtt_client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)

        # Set username and password
        self.mqtt_client.username_pw_set(mqtt_username, mqtt_password)

        # Set additional callbacks for better visibility
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_publish = lambda client, userdata, mid, properties=None: logging.info(f"Message published with mid: {mid}")
        self.mqtt_client.on_subscribe = lambda client, userdata, mid, granted_qos, properties=None: logging.info(f"Subscribed with mid: {mid}, QoS: {granted_qos}")

        # Connect to the broker
        logging.warning("Attempting to connect to MQTT broker with TLS...")
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60, clean_start=True)
        logging.warning("Connection initiated. Starting loop...")
        self.mqtt_client.loop_start()
    
    def on_connect(self, client, userdata, flags, rc, properties):
        logging.warning(f"Connected to MQTT broker with result code {rc}")
        if rc == 0:
            logging.warning("Connection successful. Subscribing to topics...")
            client.subscribe("telescope/AZEL")
        else:
            logging.error(f"Connection failed with result code {rc}")

    def on_message(self, client, userdata, msg):
        logging.warn(f"Received MQTT message on topic {msg.topic}: {msg.payload.decode()}")
        if msg.topic == "telescope/AZEL":
            try:
                az, el = map(float, msg.payload.decode().split(','))
                response = self.point(az, el)
                client.publish("telescope/response", response if response else "Error: Unable to point telescope")
            except ValueError:
                logging.error("Invalid AZEL command format. Expected 'az,el'")
                client.publish("telescope/response", "Error: Invalid AZEL command format")

    def startup(self):
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
        for cmd, pattern in seq:
            resp = self.send_command(cmd, send_delay=0.008, recv_delay=0.105)
            if pattern:
                if parse(pattern, resp) is None:
                    logging.error('startup sequence - no response')
        logging.warning('startup sequence complete')

    def send_command(self, cmd, send_delay=0.008, recv_delay=0.008):
        if isinstance(cmd, str): cmd = cmd.encode('utf-8')
        if cmd.endswith(b'\r') is False: cmd += b'\r'
        self.serial.write(cmd)
        logging.info('Sent: {}'.format(cmd))
        sleep(send_delay)
        response = self.serial.read_until(b'\r')
        response = response.strip().decode('utf-8')
        logging.info('Recv: {}'.format(response))
        sleep(recv_delay)
        if not response:
            self.serial.write(b'\r')
            sleep(send_delay)
            response = self.serial.read_until(b'\r')
            response = response.strip().decode('utf-8')
            if not response:
                logging.error('no response')
                response = ''
        # self.serial.write(b'\r')
        # self.serial.read_until(b'\r')
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
        self.brakes_off()
        self.send_command('TON')
        if not self.calibrated:
            logging.error('cannot point telescope, it is not calibrated')
            return
        if el < 15:
            logging.error('requested location is below horizon (min 15 deg elevation)')
            return
        if el > 81:
            logging.error('requested object is higher than zenith (max 81 deg elevation)')
            return
        return self.send_command(f"AZL,{az},{el}")
    
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
        self.brakes_off()
        if not self.calibrated:
            logging.warning('telescope is not calibrated, only moving elevation')
            return self.send_command('ELV,1200')
        return self.point(179,81)
    
    def maint(self): # parks dish at maintenance coordinates
        self.brakes_off()
        if not self.calibrated:
            logging.warning('telescope is not calibrated, only moving elevation')
            return self.send_command('ELV,-1200')
        return self.point(90,15)
    
    def cw(self, time=-1, speed=500):
        self.brakes_off()
        self.send_command(f"AZV,{abs(speed)}")
        if time>0: 
            sleep(time)
            self.send_command('AZV,0')
        
    def ccw(self, time=-1, speed=500):
        self.brakes_off()
        self.send_command(f"AZV,{-abs(speed)}")
        if time>0: 
            sleep(time)
            self.send_command('AZV,0')
                    
    def up(self, time=-1, speed=1100):
        self.brakes_off()
        self.send_command(f"ELV,{abs(speed)}")
        if time>0: 
            sleep(time)
            self.send_command('ELV,0')
    
    def down(self, time=-1, speed=1100):
        self.brakes_off()
        self.send_command(f"ELV,{-abs(speed)}")
        if time>0: 
            sleep(time)
            self.send_command('ELV,0')

    def get_info(self):
        # self.serial.write(b'\r')
        # self.serial.read_until(b'\r')
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
                logging.info('{}: {}'.format(k, v))

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
            
        sic1 = parse('2A01{current:d}', self.send_command('2A01SIC')) or {'current': -999999}
        sia1 = parse('2A01{current:d}', self.send_command('2A01SIA')) or {'current': -999999}
        sic2 = parse('2A02{current:d}', self.send_command('2A02SIC')) or {'current': -999999}
        sia2 = parse('2A02{current:d}', self.send_command('2A02SIA')) or {'current': -999999}
        sic3 = parse('2A03{current:d}', self.send_command('2A03SIC')) or {'current': -999999}
        sia3 = parse('2A03{current:d}', self.send_command('2A03SIA')) or {'current': -999999}
        self.amp_currents = {
            '2A01': {
                'commanded': sic1['current'],
                'actual': sia1['current'],
            },
            '2A02': {
                'commanded': sic2['current'],
                'actual': sia2['current'],
            },
            '2A03': {
                'commanded': sic3['current'],
                'actual': sia3['current'],
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
        for cmd, pattern in seq:
            resp = self.send_command(cmd, send_delay=0.008, recv_delay=0.105)
            if pattern:
                if parse(pattern, resp) is None:
                    logging.error('calibration bad response: {}'.format(resp))
                    return
        self.get_info()
        while self.CalSts != 'Calibration OK':
            sleep(0.5)
            self.get_info()
        logging.warning('calibration complete')
        self.send_command('TMD,0')
        self.send_command('TON')
                
    def spa(self):
        self.send_command('SPA')

    def stop(self):
        self.send_command('SPA')
        
    def cleanup(self):
        self.spa()
        self.brakes_on()
        self.serial.close()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def point_at_CasA(self):
        t = Time.now()
        print(t)
        try:
            while True:
                iterate = 0
                t = Time.now()
                obj = FixedTarget.from_name("Caph")
                altaz = self.obs.altaz(t, obj)
                az = altaz.az.deg
                el = altaz.alt.deg
                while iterate <= 3:
                    self.point_object("Caph")
                    sleep(10)
                    iterate += 1
                self.point(az+10, el)
                sleep(30)
        except KeyboardInterrupt:
            print('interrupted!')
            print(t)
        
        
if __name__ == '__main__':
    c = Caltech6m(verbose=True, mqtt_broker='0fed07f982184f4db2a5cbd8f181ccae.s1.eu.hivemq.cloud', mqtt_port=8883, mqtt_username='scp_mtf_dei_pda', mqtt_password='Delta-42')
    try:
        while True:
            cmd = input('>>> ')
            if cmd == 'exit':
                break
            if not cmd:
                continue
            if not cmd.startswith('c.'):
                cmd = 'c.'+cmd
            if not cmd.endswith(')') and len(cmd.split(' ')) == 1:
                cmd += '()'
            if len(cmd.split(' ')) > 1:
                cmd = f'{cmd.split(" ")[0]}({",".join(cmd.split(" ")[1:])})'
            try:
                print(cmd)
                exec(cmd)
            except Exception as e:
                print(e)
    finally:
        c.cleanup()