import serial
ser = serial.Serial(
    port='COM16',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.5,
)

from astropy.coordinates import SkyCoord, get_sun, EarthLocation, AltAz
from astropy.time import Time

# get ra, dec of sun in pasadena right now
t = Time.now()
c = SkyCoord(get_sun(t), frame='gcrs', obstime=t, location=EarthLocation.from_geodetic('-118d7.4650m', '34d8.3860m', '204.7m'))
print(c.ra.hms, c.dec.dms)

c = c.transform_to(AltAz(obstime=t, location=EarthLocation.from_geodetic('-118d7.4650m', '34d8.3860m', '204.7m')))
print(c.az.degree,c.alt.degree)