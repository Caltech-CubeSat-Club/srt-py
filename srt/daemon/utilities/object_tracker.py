"""object_tracker.py

Module for Tracking and Caching the Azimuth-Elevation Coords of Celestial Objects

"""
from astropy.coordinates import SkyCoord, EarthLocation, get_body, get_sun
from astropy.coordinates import Galactic, CIRS, AltAz
from astropy.utils.iers.iers import conf
from astropy.table import Table
from astropy.time import Time
import astropy.units as _u

import numpy as np
from pathlib import Path
from typing import Any, cast


deg: Any = getattr(_u, "deg")
hourangle: Any = getattr(_u, "hourangle")
km: Any = getattr(_u, "km")
m: Any = getattr(_u, "m")
s: Any = getattr(_u, "s")
second: Any = getattr(_u, "second")


root_folder = Path(__file__).parent.parent.parent.parent


class EphemerisTracker:
    """
    Enables Calculating the AzEl Coordinates of the Bodies Specified in sky_coords.csv
    """

    def __init__(
        self,
        observer_lat,
        observer_lon,
        observer_elevation=0,
        config_file="config/sky_coords.csv",
        refresh_time=10,
        auto_download=True,
    ):
        """Initializer for EphemerisTracker

        - Reads CSV File for Objects to Track
        - Converts to Common Coordinate System
        - Populates AzEl Dictionary with Current Values

        Parameters
        ----------
        observer_lat : float
            Observer's Location Latitude in degrees
        observer_lon : float
            Observer's Location Longitude in degrees
        observer_elevation : float
            Observer's Location Elevation in meters
        config_file : str
            Location of the List File for Bodies Being Tracked
        refresh_time : float
            Maximum Amount of Time Cache is Valid
        auto_download : bool
            Whether AstroPy is Permitted to Use Internet to Increase Accuracy
        """

        table = Table.read(Path(root_folder, config_file), format="ascii.csv")

        self.sky_coord_names = {}
        sky_coords_ra = np.zeros(len(table))
        sky_coords_dec = np.zeros(len(table))

        for index, row in enumerate(table):
            coordinate_system = row["coordinate_system"]
            coordinate_a = row["coordinate_a"]
            coordinate_b = row["coordinate_b"]
            name = row["name"]
            unit = (
                deg
                if coordinate_system == Galactic.__name__.lower()
                else (hourangle, deg)
            )
            sky_coord = SkyCoord(
                coordinate_a, coordinate_b, frame=coordinate_system, unit=unit
            )
            sky_coord_transformed = cast(Any, sky_coord.transform_to(CIRS))
            sky_coords_ra[index] = sky_coord_transformed.ra.degree
            sky_coords_dec[index] = sky_coord_transformed.dec.degree
            self.sky_coord_names[name] = index

        self.sky_coords = SkyCoord(
            ra=sky_coords_ra * deg, dec=sky_coords_dec * deg, frame=CIRS
        )
        self.location = EarthLocation.from_geodetic(
            lat=observer_lat * deg,
            lon=observer_lon * deg,
            height=observer_elevation * m,
        )
        self.latest_time = None
        self.refresh_time = refresh_time * second

        self.az_el_dict = {}
        self.vlsr_dict = {}
        # self.time_interval_dict = {}
        self.time_offsets_seconds = sorted(
            set(list(range(0, 61, 5)) + [3600, 7200, 10800, 14400, 18000, 21600])
        )
        self.time_interval_dict = self.initial_azeltime()

        # Configure astropy download behavior before first ephemeris update.
        conf.auto_download = auto_download

        self.update_all_az_el()
        # self.update_azeltime()

    def calculate_az_el(self, name, time, alt_az_frame):
        """Calculates Azimuth and Elevation of the Specified Object at the Specified Time

        Parameters
        ----------
        name : str
            Name of the Object being Tracked
        time : Time
            Current Time (only necessary for Sun/Moon Ephemeris)
        alt_az_frame : AltAz
            AltAz Frame Object

        Returns
        -------
        (float, float)
            (az, el) Tuple
        """
        if name == "Sun":
            alt_az = cast(Any, get_sun(time).transform_to(alt_az_frame))
        elif name == "Moon":
            alt_az = cast(
                Any, get_body("moon", time, self.location).transform_to(alt_az_frame)
            )
        elif name == "Jupiter":
            alt_az = cast(
                Any, get_body("jupiter", time, self.location).transform_to(alt_az_frame)
            )
        else:
            target_coord = cast(Any, self.sky_coords[self.sky_coord_names[name]])
            alt_az = cast(
                Any,
                target_coord.transform_to(alt_az_frame),
            )
        return alt_az.az.degree, alt_az.alt.degree

    def calculate_vlsr(self, name, time, frame):
        """Calculates the velocity in the local standard of rest.

        Parameters
        ----------
        name : str
            Name of the Object being Tracked
        time : Time
            Current Time (only necessary for Sun/Moon Ephemeris)
        alt_az_frame : AltAz
            AltAz Frame Object


        Returns
        -------
        float
            vlsr in km/s.
        """
        if name == "Sun":
            tframe = cast(Any, get_sun(time).transform_to(frame))
            vlsr = tframe.radial_velocity_correction(obstime=time)
        elif name == "Moon":
            tframe = cast(
                Any, get_body("moon", time, self.location).transform_to(frame)
            )
            vlsr = tframe.radial_velocity_correction(obstime=time)
        elif name == "Jupiter":
            tframe = cast(
                Any, get_body("jupiter", time, self.location).transform_to(frame)
            )
            vlsr = tframe.radial_velocity_correction(obstime=time)
        else:
            target_coord = cast(Any, self.sky_coords[self.sky_coord_names[name]])
            tframe = cast(
                Any,
                target_coord.transform_to(frame),
            )
            vlsr = tframe.radial_velocity_correction(obstime=time)

        return vlsr.to(km / s).value

    def calculate_vlsr_azel(self, az_el, time=None):
        """Takes an AzEl tuple and derives the vlsr from  Location

        Parameters
        ----------
        az_el : (float, float)
            Azimuth and Elevation
        time : AstroPy Time Obj
            Time of Conversion

        Returns
        -------
        float
            vlsr in km/s.
        """

        if time is None:
            time = Time.now()

        az, el = az_el
        start_frame = AltAz(
            obstime=time, location=self.location, alt=el * deg, az=az * deg
        )
        end_frame = Galactic()
        result = start_frame.transform_to(end_frame)
        sk1 = SkyCoord(result)
        f1 = AltAz(obstime=time, location=self.location)
        vlsr = sk1.transform_to(f1).radial_velocity_correction(obstime=time)

        return vlsr.to(km / s).value

    def convert_to_gal_coord(self, az_el, time=None):
        """Converts an AzEl Tuple into a Galactic Tuple from Location

        Parameters
        ----------
        az_el : (float, float)
            Azimuth and Elevation to Convert
        time : AstroPy Time Obj
            Time of Conversion

        Returns
        -------
        (float, float)
            Galactic Latitude and Longitude
        """
        if time is None:
            time = Time.now()
        az, el = az_el
        start_frame = AltAz(
            obstime=time, location=self.location, alt=el * deg, az=az * deg
        )
        end_frame = Galactic()
        result = start_frame.transform_to(end_frame)
        g_lat = float(result.b.degree)
        g_lng = float(result.l.degree)
        return g_lat, g_lng

    def update_all_az_el(self):
        """Updates Every Entry in the AzEl Dictionary Cache, if the Cache is Outdated

        Returns
        -------
        None
        """
        if (
            self.latest_time is not None
            and Time.now() < self.latest_time + self.refresh_time
        ):
            return
        time = Time.now()
        frame = AltAz(obstime=time, location=self.location)
        transformed = cast(Any, self.sky_coords.transform_to(frame))
        for name in self.sky_coord_names:
            index = self.sky_coord_names[name]
            self.az_el_dict[name] = (
                transformed.az[index].degree,
                transformed.alt[index].degree,
            )
            vlsr = transformed[index].radial_velocity_correction(obstime=time)
            self.vlsr_dict[name] = vlsr.to(km / s).value
        self.az_el_dict["Sun"] = self.calculate_az_el("Sun", time, frame)
        self.vlsr_dict["Sun"] = self.calculate_vlsr("Sun", time, frame)
        self.az_el_dict["Moon"] = self.calculate_az_el("Moon", time, frame)
        self.vlsr_dict["Moon"] = self.calculate_vlsr("Moon", time, frame)
        self.az_el_dict["Jupiter"] = self.calculate_az_el("Jupiter", time, frame)
        self.vlsr_dict["Jupiter"] = self.calculate_vlsr("Jupiter", time, frame)

        for time_passed in self.iter_time_offsets_seconds():

            timenew = time + time_passed * second
            frame = AltAz(obstime=timenew, location=self.location)
            transformed = cast(Any, self.sky_coords.transform_to(frame))

            for name in self.sky_coord_names:
                index = self.sky_coord_names[name]
                self.time_interval_dict[time_passed][name] = (
                    transformed.az[index].degree,
                    transformed.alt[index].degree,
                )
            self.time_interval_dict[time_passed]["Sun"] = self.calculate_az_el(
                "Sun", timenew, frame)
            self.time_interval_dict[time_passed]["Moon"] = self.calculate_az_el(
                "Moon", timenew, frame)
            self.time_interval_dict[time_passed]["Jupiter"] = self.calculate_az_el(
                "Jupiter", timenew, frame)

        self.latest_time = time

    def get_all_azimuth_elevation(self):
        """Returns Dictionary Mapping the Objects to their Current AzEl Coordinates

        Returns
        -------
        self.az_el_dict : {str: (float, float)}
        """
        return self.az_el_dict

    def get_all_azel_time(self):
        """Returns Dictionary Mapping the Time Offset to a dictionary of updated azel coordinates

        Returns
        -------
        self.time_interval_dict : {int: {str: (float, float)}}s
        """
        # return
        return self.time_interval_dict

    def get_azimuth_elevation(self, name, time_offset):
        """Returns Individual Object AzEl at Specified Time Offset

        Parameters
        ----------
        name : str
            Object Name
        time_offset : Time
            Any Offset from the Current Time
        Returns
        -------
        (float, float)
            (az, el) Tuple
        """
        if time_offset == 0:
            return self.get_all_azimuth_elevation()[name]
        else:
            time = Time.now() + time_offset * second
            return self.calculate_az_el(
                name, time, AltAz(obstime=time, location=self.location)
            )

    def get_all_vlsr(self):
        return self.vlsr_dict

    def get_vlsr(self, name, time_offset=0):

        if time_offset == 0:
            return self.get_all_vlsr()[name]
        else:
            time = Time.now() + time_offset * second
            frame = AltAz(obstime=time, location=self.location)
            return self.calculate_vlsr(name, time, frame)

    def iter_time_offsets_seconds(self):
        """Iterate configured future time offsets in seconds."""
        return tuple(self.time_offsets_seconds)

    def initial_azeltime(self):
        new_dict = {}
        for time_passed in self.iter_time_offsets_seconds():
            # new_time_dict = deepcopy(self.az_el_dict)
            new_time_dict = {}
            new_dict[time_passed] = new_time_dict
        return new_dict

    def inital_azeltime(self):
        """Backward-compatible alias for initial_azeltime."""
        return self.initial_azeltime()

    def update_azeltime(self):
        # if (
        #     self.latest_time is not None
        #     and Time.now() < self.latest_time + self.refresh_time
        # ):
        #     return

        for time_passed in self.iter_time_offsets_seconds():

            time = Time.now() + time_passed * second
            frame = AltAz(obstime=time, location=self.location)
            transformed = cast(Any, self.sky_coords.transform_to(frame))

            for name in self.sky_coord_names:
                index = self.sky_coord_names[name]
                self.time_interval_dict[time_passed][name] = (
                    transformed.az[index].degree,
                    transformed.alt[index].degree,
                )
        self.latest_time = Time.now()
        return
