"""motors.py

Module for Controlling Different Motor Types over Serial

"""
import serial

from abc import ABC, abstractmethod
from time import sleep
from math import cos, acos, pi, sqrt, floor
from parse import parse, search


class Motor(ABC):
    """Abstract Class for All Motors Types

    Attributes
    ----------
    port : str
        Serial Port Identifier String for Communicating with the Motor
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

    def __init__(self, port, az_limits, el_limits):
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
        self.az_limits = az_limits
        self.el_limits = el_limits
        self.serial = None

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

    def __init__(self, port, az_limits, el_limits):
        """
        Initializer for Rot2Motor

        Parameters
        ----------
        port : str
            NOT USED - Needed For Abstract Motor Initializer
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits (if Stationary, both should be the same value)
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits (if Stationary, both should be the same value)
        """
        super().__init__(port, az_limits, el_limits)
        self.position = (az_limits[0], el_limits[0])

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
        self.position = (az, el)

    def status(self):
        """Returns the Unchanging Position of the Stationary / Simulated Motor

        Returns
        -------
        (float, float)
            Current Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        return self.position


class Rot2Motor(Motor):
    """Class for Controlling any ROT2 Protocol-Supporting Motor (e.g. SPID Motors)

    See Also
    --------
    <http://ryeng.name/blog/3>
    <https://github.com/jaidenfe/rot2proG/blob/master/rot2proG.py>
    <https://www.haystack.mit.edu/edu/undergrad/srt/pdf%20files/MD-01%20en.pdf>
    """

    VALID_PULSES_PER_DEGREE = (1, 2, 4)

    def __init__(
        self,
        port,
        az_limits,
        el_limits,
        pulses_per_degree=2,
        test_pulses_per_degree=True,
    ):
        """Initializer for Rot2Motor

        Parameters
        ----------
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        pulses_per_degree : int
            Number of Motor Pulses per Degree of Movement
        test_pulses_per_degree : bool
            Whether to Run A Call to Ask the Motor What its True Pulses per Degree Is (By Calling status)
        """
        Motor.__init__(self, port, az_limits, el_limits)
        self.serial = serial.Serial(
            port=self.port,
            baudrate=600,
            bytesize=serial.EIGHTBITS,
            parity="N",
            stopbits=serial.STOPBITS_ONE,
            timeout=None,
        )
        if pulses_per_degree in Rot2Motor.VALID_PULSES_PER_DEGREE:
            self.pulses_per_degree = pulses_per_degree
        else:
            raise ValueError("Invalid Pulse Per Degree Value")
        if test_pulses_per_degree:
            self.status()

    def send_rot2_pkt(self, cmd, az=None, el=None):
        """Builds and Sends a ROT2 Command Packet over Serial

        Parameters
        ----------
        cmd : int
            ROT2 Motor Command Value (0x2F -> Set, 0x1F -> Get, 0x0F -> Stop)
        az : float
            Azimuth Coordinate to Point At (If Applicable)
        el : float
            Elevation Coordinate to Point At (If Applicable)

        Notes
        -----
        All send_rot2_pkt calls should be followed with a receive_rot2_pkt

        Returns
        -------
        None
        """
        if az is not None and el is not None:
            azimuth = int(
                self.pulses_per_degree * (az + 360.0) + 0.5
            )  # Formatted Az Pulse Value
            elevation = int(
                self.pulses_per_degree * (el + 360.0) + 0.5
            )  # Formatted El Pulse Value
        else:
            azimuth = 0
            elevation = 0

        azimuth_ticks = (
            self.pulses_per_degree
        )  # Documentation for Rot2 Says This Is Ignored
        elevation_ticks = (
            self.pulses_per_degree
        )  # Documentation for Rot2 Says This Is Ignored

        cmd_string = "W%04d%c%04d%c%c " % (
            azimuth,
            azimuth_ticks,
            elevation,
            elevation_ticks,
            cmd,
        )
        cmd_bytes = cmd_string.encode("ascii")
        # print("Packet of Size " + str(len(cmd_bytes)))
        # print([hex(val) for val in cmd_bytes])
        self.serial.write(cmd_bytes)

    def receive_rot2_pkt(self):
        """Receives and Parsers an ROT2 Status Packet

        Returns
        -------
        (float, float)
            Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        received_vals = self.serial.read(12)
        az = (
            (received_vals[1] * 100)
            + (received_vals[2] * 10)
            + received_vals[3]
            + (received_vals[4] / 10.0)
            - 360.0
        )
        el = (
            (received_vals[6] * 100)
            + (received_vals[7] * 10)
            + received_vals[8]
            + (received_vals[9] / 10.0)
            - 360.0
        )
        az_pulse_per_deg = received_vals[5]
        el_pulse_per_deg = received_vals[10]
        assert az_pulse_per_deg == el_pulse_per_deg  # Consistency Check
        if az_pulse_per_deg != self.pulses_per_degree:
            print(
                "Motor Pulses Per Degree Incorrect, Changing Value to "
                + str(az_pulse_per_deg)
            )
            self.pulses_per_degree = az_pulse_per_deg
        return az, el

    def point(self, az, el):
        """Point ROT2 Motor at AzEl Coordinate

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
        cmd = 0x2F  # Rot2 Set Command
        az_relative = az - self.az_limits[0]
        el_relative = el - self.el_limits[0]
        self.send_rot2_pkt(cmd, az=az_relative, el=el_relative)

    def status(self):
        """Requests the Current Location of the ROT2 Motor

        Returns
        -------
        (float, float)
            Current Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        cmd = 0x1F  # Rot2 Status Command
        self.send_rot2_pkt(cmd)
        az_relative, el_relative = self.receive_rot2_pkt()
        return az_relative + self.az_limits[0], el_relative + self.el_limits[0]

    def stop(self):
        """Stops the ROT2 Motor at its Current Location

        Returns
        -------
        None
        """
        cmd = 0x0F  # Rot2 Stop Command
        self.send_rot2_pkt(cmd)
        # az_relative, el_relative = self.receive_rot2_pkt()
        # return (az_relative + self.az_limits[0], el_relative + self.el_limits[0])


class H180Motor(Motor):  # TODO: Test!
    """
    Class for Controlling any ROT2 Protocol-Supporting Motor (e.g. SPID Motors)
    Based on the h180 function from the C SRT code

    ftp://gemini.haystack.mit.edu/pub/web/src/source_srt_newsrtsource_ver9.tar.gz
    """

    AZCOUNTS_PER_DEG = 52.0 * 27.0 / 120.0
    ELCOUNTS_PER_DEG = 52.0 * 27.0 / 120.0

    def __init__(self, port, az_limits, el_limits, counts_per_step=100):
        """Initializer for the H180 Motor

        Parameters
        ----------
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        counts_per_step : int
            Maximum number of counts to move per call to function
        """
        Motor.__init__(self, port, az_limits, el_limits)
        self.serial = serial.Serial(
            port=port,
            baudrate=2400,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=None,
        )
        self.count_per_step = counts_per_step
        self.az_lower_lim = az_limits[0]
        self.el_lower_lim = el_limits[0]
        self.az_count = 0.0
        self.el_count = 0.0

    def send_h180_cmd(self, az, el, stow):
        """Sends a Command to the H180 Motor

        Parameters
        ----------
        az : float
            Azimuth Coordinate to Point At
        el : float
            Elevation Coordinate to Point At
        stow : bool
            Whether or Not to Stow Antenna (makes az,el irrelevant)

        Returns
        -------
        None
        """
        azz = az - self.az_lower_lim
        ell = el - self.el_lower_lim
        for axis in range(2):
            mm = -1
            count = 0
            if stow:
                if axis == 0:
                    mm = 0
                else:
                    mm = 2
                count = 8000
            else:
                if axis == 0:
                    acount = azz * H180Motor.AZCOUNTS_PER_DEG - self.az_count
                    if self.count_per_step and acount > self.count_per_step:
                        acount = self.count_per_step
                    if self.count_per_step and acount < -self.count_per_step:
                        acount = -self.count_per_step
                    if acount > 0:
                        count = acount + 0.5
                    else:
                        count = acount - 0.5
                    if count > 0:
                        mm = 1
                    if count < 0:
                        mm = 0
                if axis == 1:
                    acount = ell * H180Motor.ELCOUNTS_PER_DEG - self.el_count
                    if self.count_per_step and acount > self.count_per_step:
                        acount = self.count_per_step
                    if self.count_per_step and acount < -self.count_per_step:
                        acount = -self.count_per_step
                    if acount > 0:
                        count = acount + 0.5
                    else:
                        count = acount - 0.5
                    if count > 0:
                        mm = 3
                    if count < 0:
                        mm = 2
                if count < 0:
                    count = -count
            if mm >= 0 and count:
                cmd_string = " move %d %d%1c" % (mm, count, 13)
                self.serial.write(cmd_string.encode("ascii"))
                resp = ""
                sleep(0.01)
                im = 0
                i = 0
                while i < 32:
                    ch = int.from_bytes(self.serial.read(1), byteorder="big")
                    sleep(0.01)
                    if i < 32:
                        resp += chr(ch)
                        i += 1
                    if ch == 13 or ch == 10:
                        break
                status = i
                sleep(0.1)
                for i in range(status):
                    if resp[i] == "M" or resp[i] == "T":
                        im = i
                ccount = int(resp[im:status].split(" ")[-1])
                if resp[im] == "M":
                    if mm == 1:
                        self.az_count += ccount
                    if mm == 0:
                        self.az_count -= ccount
                    if mm == 3:
                        self.el_count += ccount
                    if mm == 2:
                        self.el_count -= ccount
                if resp[im] == "T":
                    if mm == 1:
                        self.az_count += count
                    if mm == 0:
                        self.az_count -= count
                    if mm == 3:
                        self.el_count += count
                    if mm == 2:
                        self.el_count -= count
        if stow:
            self.az_count = 0
            self.el_count = 0

    def point(self, az, el):
        """Points an H180 Motor at a Certain Az, El

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
        self.send_h180_cmd(az, el, False)
        return self.status()

    def status(self):
        """Requests the Current Location of the H180 Motor

        Returns
        -------
        (float, float)
            Current Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        azz = self.az_count / H180Motor.AZCOUNTS_PER_DEG
        ell = self.el_count / H180Motor.ELCOUNTS_PER_DEG
        az = azz + self.az_lower_lim
        el = ell + self.el_lower_lim
        return az, el


class PushRodMotor(Motor):  # TODO: Test!
    """
    Controls old SRT PushRod Style Motors

    WARNING: This is currently a hard port of the azel function in sport.java, so expect some errors
    """

    AZCOUNTS_PER_DEG = (
        8.0 * 32.0 * 60.0 / (360.0 * 9.0)
    )  # Should this be 52.0 * 27.0 / 120.0?
    ELCOUNTS_PER_DEG = 52.0 * 27.0 / 120.0

    def __init__(
        self,
        port,
        az_limits,
        el_limits,
        rod=(),
        counts_per_step=100,
        count_tol=1,
        count_corr=(0, 0),
    ):
        """

        Parameters
        ----------
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        """
        Motor.__init__(self, port, az_limits, el_limits)
        self.serial = serial.Serial(
            port=port,
            baudrate=2000,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        self.rod = rod
        self.az_count = 0.0
        self.el_count = 0.0
        self.count_per_step = counts_per_step
        self.count_tol = count_tol
        self.count_corrections = count_corr
        self.az = az_limits[0]
        self.el = el_limits[0]
        self.azatstow = 0
        self.elatstow = 0

    def send_pushrod_cmd(self, az, el, stow):
        """Sends a Command to the Pushrod Motor

        Parameters
        ----------
        az : float
            Azimuth Coordinate to Point At
        el : float
            Elevation Coordinate to Point At
        stow : bool
            Whether or Not to Stow Antenna (makes az, el irrelevant)

        Returns
        -------
        None
        """
        mm = count = 0
        lenzero = 0.0

        az = az % 360  # put az into reasonable range
        az = az + 360.0  # put az in range 180 to 540
        if az > 540.0:
            az -= 360.0
        if az < 180.0:
            az += 360.0

        region1 = region2 = region3 = 0
        if (
            self.az_limits[0] <= az < self.az_limits[1]
            and self.el_limits[0] <= el <= self.el_limits[1]
        ):
            region1 = 1
        if az > self.az_limits[0] + 180.0 and el > (180.0 - self.el_limits[1]):
            region2 = 1
        if az < self.az_limits[1] - 180.0 and el > (180.0 - self.el_limits[1]):
            region3 = 1
        if region1 == 0 and region2 == 0 and region3 == 0:
            raise ValueError("The Azimuth and Elevation Provided are Not Valid")

        flip = 0
        azz = az - self.az_limits[0]
        ell = el - self.el_limits[0]
        azscale = self.AZCOUNTS_PER_DEG
        elscale = self.ELCOUNTS_PER_DEG
        # g.set_slew(0);

        lenzero = (
            self.rod[0] * self.rod[0]
            + self.rod[1] * self.rod[1]
            - 2.0
            * self.rod[0]
            * self.rod[1]
            * cos((self.rod[3] - self.el_limits[0]) * pi / 180.0)
            - self.rod[2] * self.rod[2]
        )
        if lenzero >= 0.0:
            lenzero = sqrt(lenzero)
        else:
            lenzero = 0

        ellcount = (
            self.rod[0] * self.rod[0]
            + self.rod[1] * self.rod[1]
            - 2.0 * self.rod[0] * self.rod[1] * cos((self.rod[3] - el) * pi / 180.0)
            - self.rod[2] * self.rod[2]
        )
        if ellcount >= 0.0:
            ellcount = (-sqrt(ellcount) + lenzero) * self.rod[4]
        else:
            ellcount = 0

        if ellcount > self.el_count * 0.5:
            axis = 1
        else:
            axis = 0

        for ax in range(0, 2):
            if axis == 0:
                if azz * azscale > self.az_count * 0.5 - 0.5:
                    mm = 1
                    count = int(floor(azz * azscale - self.az_count * 0.5 + 0.5))
                if azz * azscale <= self.az_count * 0.5 + 0.5:
                    mm = 0
                    count = int(floor(self.az_count * 0.5 - azz * azscale + 0.5))
            else:
                if ellcount > self.el_count * 0.5 - 0.5:
                    mm = 3
                    count = int(floor(ellcount - self.el_count * 0.5 + 0.5))
                if ellcount <= self.el_count * 0.5 + 0.5:
                    mm = 2
                    count = int(floor(self.el_count * 0.5 - ellcount + 0.5))
            ccount = count
            if stow == 1:  # drive to stow
                count = 5000
                if axis == 0:
                    mm = 0
                    if self.azatstow == 1:
                        count = 0
                if axis == 1:
                    mm = 2  # complete azimuth motion to stow before completely drop in elevation
                    if self.elatstow == 1 or (
                        ccount <= 2.0 * self.count_per_step and self.azatstow == 0
                    ):
                        count = 0
                flip = 0
            if count > self.count_per_step and ccount > self.count_per_step:
                count = self.count_per_step
            if count >= self.count_tol:
                cmd_str = (
                    "  move " + str(mm) + " " + str(count) + "\n"
                )  # need space at start and end
                n = 0
                if count < 5000:
                    str2 = "M " + str(count) + "\n"
                else:
                    str2 = "T " + str(count) + "\n"
                recv = str2
                n = len(str2)
                j = 0
                kk = -1
                try:
                    self.serial.write(cmd_str.encode("ascii"))
                    j = n = rcount = kk = 0
                    resp = ""
                    while 0 <= kk < 3000:
                        result = self.serial.read(1)
                        if len(result) < 1:
                            j = -1
                        else:
                            j = int.from_bytes(result, byteorder="big")
                        kk += 1
                        if j >= 0 and n < 80:
                            resp += chr(j)
                            n += 1
                        if n > 0 and j == -1:
                            kk = -1  # end of message
                        # t.getTsec(g, d, gg)
                    recv = resp
                except Exception as e:
                    print(e)
                if kk != -1 or (recv[0] != "M" and recv[0] != "T"):
                    print("* ERROR comerr")
                    return  # TODO: Should throw error here?
                sleep(0.1)
                cmd_str = recv[0:n]  # String.copyValueOf(recv, 0, n - 1)
                parsed_strings = cmd_str.split(" ")
                try:
                    str2 = parsed_strings[0]
                except IndexError as e:
                    print(e)
                rcount = 0
                try:
                    srt2 = parsed_strings[1]
                    rcount = int(str2)
                except IndexError as e:
                    print(e)
                b2count = 0
                try:
                    str2 = parsed_strings[2]
                    b2count = int(str2)
                except IndexError as e:
                    print(e)
                if count < 5000:
                    fcount = (
                        count * 2 + b2count
                    )  # add extra 1 / 2 count from motor coast
                else:
                    fcount = 0
                if mm == 2 and recv[0] == "T":
                    self.elatstow = 1
                    self.el_count = 0
                if mm == 0 and recv[0] == "T":
                    self.azatstow = 1
                    self.az_count = 0
                if recv[0] == "M":
                    if axis == 0:
                        self.azatstow = 0
                        if mm == 1:
                            self.az_count += fcount
                        else:
                            self.az_count -= fcount
                    if axis == 1:
                        self.elatstow = 0
                        if mm == 3:
                            self.el_count += fcount
                        else:
                            self.el_count -= fcount
                sleep(0.005)
            axis += 1
            if axis > 1:
                axis = 0
        self.az = (
            self.az_limits[0]
            - self.count_corrections[0]
            + self.az_count * 0.5 / azscale
        )
        if self.az > 360.0:
            self.az -= 360.0
        ellnow = -self.el_count * 0.5 / self.rod[4] + lenzero
        ellnow = (
            self.rod[0] * self.rod[0]
            + self.rod[1] * self.rod[1]
            - self.rod[2] * self.rod[2]
            - ellnow * ellnow
        )
        ellnow = ellnow / (2.0 * self.rod[0] * self.rod[1])
        ellnow = -acos(ellnow) * 180.0 / pi + self.rod[3] - self.el_limits[0]
        self.el = self.el_limits[0] - self.count_corrections[1] + ellnow
        if self.el > 90.0:
            if self.az >= 180.0:
                self.az -= 180.0
            else:
                self.az += 180.0
                self.el = 180.0 - self.el

    def point(self, az, el):
        """Points an Pushrod Motor at a Certain Az, El

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
        self.send_pushrod_cmd(az, el, 0)

    def status(self):
        """Requests the Current Location of the Pushrod Motor

        Returns
        -------
        (float, float)
            Current Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        return self.az, self.el

class Caltech6m(Motor):
    """

    """
    # 5000 line encoder, quadrature encoded (X4) = 20000 cnts/rev
    # az and el motor deg per motor encoder count
    # AZCOUNTS_PER_DEG = 20000
    # ELCOUNTS_PER_DEG = 20000
    # LPR,0.4,0.4,2.0,2.0,0.01,0.01,1.5,1.0,2.5,1.0,1.0,5.0,2.0,1.2,1.0,1.0,0.8,1.5,0.0,0.0,1.0,1.0,800,1200,-800,-1300,1.0,1.0,150,144000,144000,20000,20000,-301.087,118.5,1522.5,19749

    def __init__(
        self,
        port,
        az_limits,
        el_limits
    ):
        """

        Parameters
        ----------
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        """
        Motor.__init__(self, port, az_limits, el_limits)
        self.serial = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.5,
        )
        self.error_state = 'NONE'
        self.az = az_limits[0]
        self.el = el_limits[0]
        self.azatstow = 179
        self.elatstow = 81
        self.mode = 'Stop'
        self.ElUpPreLim = False
        self.ElDnPreLim = False
        self.ElUpFinLim = False
        self.ElDnFinLim = False
        self.AzCwPreLim = False
        self.AzCcwPreLim = False
        self.AzCwFinLim = False
        self.AzCcwFinLim = False
        self.AzLT180 = False
        self.AzBrkOn = True
        self.ElBrkOn = True
        self.EmStopOn = False
        self.CalSts = 'Not Calibrated'
        # TODO: add currently_calibrating, currently_moving, etc.
        self.startup()
        self.get_info()

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
            resp = self.serial_send_recv(cmd, send_delay=0.008, recv_delay=0.105)
            if pattern:
                if parse(pattern, resp) is None:
                    self.error_state = 'STARTUP_BAD_RESPONSE'

    
    def serial_send_recv(self, cmd, send_delay=0.008, recv_delay=0):
        if isinstance(cmd, str): cmd = cmd.encode('utf-8')
        if cmd.endswith(b'\r') is False: cmd += b'\r'
        self.serial.write(cmd)
        sleep(send_delay)
        response = self.serial.read_until(b'\r')
        response = response.strip().decode('utf-8')
        sleep(recv_delay)
        return response if response is not None else ''

    def point(self, az, el):
        # TODO: SPA, TMD,0 and TON commands after calibrating, before moving
        self.serial_send_recv('AZEL,{:.2f},{:.2f},1000'.format(az, el))

    def get_info(self):
        sts = self.serial_send_recv('STS')
        sts = parse('STS,{mode:d}{ElUpPreLim:l}{ElDnPreLim:l}{ElUpFinLim:l}{ElDnFinLim:l}{AzCwPreLim:l}{AzCcwPreLim:l}{AzCwFinLim:l}{AzCcwFinLim:l}{AzLT180:l}{AzBrkOn:l}{ElBrkOn:l}{EmStopOn:l}{CalSts:d}{idk}', sts).named
        if sts is not None:
            for k,v in sts.items():
                if isinstance(v, str):
                    if len(v) == 1:
                        sts[k] = (v == 'T')
                if isinstance(v, int):
                    if k == 'mode':
                        sts[k] = {0:'Stop', 1:'Calibrate', 5:'Track'}[v]
                    if k == 'CalSts':
                        sts[k] = {0:'Not Calibrated', 1:'Calibrating Now', 2:'Calibration OK'}[v]
            self.__dict__.update(sts)
        else:
            self.error_state = 'STS_BAD_RESPONSE'

        azel = self.serial_send_recv('GAE')
        azel = parse('GAE,{az:f},{el:f}', azel).named
        if azel is not None:
            self.__dict__.update(azel)
        else:
            self.error_state = 'GAE_BAD_RESPONSE'

    def status(self):
        self.get_info()
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
        for cmd, pattern in seq:
            resp = self.serial_send_recv(cmd, send_delay=0.008, recv_delay=0.105)
            if pattern:
                if parse(pattern, resp) is None:
                    self.error_state = 'CAL_BAD_RESPONSE'
                    
    def stop(self):
        self.serial_send_recv('SPA')