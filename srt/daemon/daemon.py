"""daemon.py

Main Control and Orchestration Class for the Small Radio Telescope

"""

from time import sleep, time
from datetime import timedelta, datetime
from threading import Thread
from queue import Queue
from collections import deque
from xmlrpc.client import ServerProxy
from pathlib import Path
from operator import add

import zmq
import json
import logging
import numpy as np

from .rotor_control.rotors import Rotor
from .utilities.object_tracker import EphemerisTracker
from .utilities.functions import azel_within_range, get_spectrum


class SmallRadioTelescopeDaemon:
    """
    Controller Class for the Small Radio Telescope
    """

    def __init__(self, config_directory, config_dict):
        """Initializer for the Small Radio Telescope Daemon

        Parameters
        ----------
        config_directory : str
            Path to the Directory Containing Configuration Files
        config_dict : dict
            Dictionary Containing SRT Settings
        """

        # Store Individual Settings In Object
        self.config_directory = config_directory
        if "STATION" in config_dict:
            self.station = config_dict["STATION"]
        else:
            self.station = {"latitude": 0.0,
                            "longitude": 0.0,
                            "name": None}
        self.contact = config_dict["EMERGENCY_CONTACT"]
        self.az_limits = (
            config_dict["AZLIMITS"]["lower_bound"],
            config_dict["AZLIMITS"]["upper_bound"],
        )
        self.el_limits = (
            config_dict["ELLIMITS"]["lower_bound"],
            config_dict["ELLIMITS"]["upper_bound"],
        )
        self.stow_location = (
            config_dict["STOW_LOCATION"]["azimuth"],
            config_dict["STOW_LOCATION"]["elevation"],
        )
        self.cal_location = (
            config_dict["CAL_LOCATION"]["azimuth"],
            config_dict["CAL_LOCATION"]["elevation"],
        )
        self.horizon_points = [
            (point["azimuth"], point["elevation"])
            for point in config_dict["HORIZON_POINTS"]
        ]
        self.motor_type = config_dict["MOTOR_TYPE"]
        self.motor_port = config_dict["MOTOR_PORT"]
        self.motor_baudrate = config_dict["MOTOR_BAUDRATE"]
        self.caltech6m_safe_mode = bool(config_dict.get("CALTECH6M_SAFE_MODE", False))
        self.control_profile = str(
            config_dict.get("CONTROL_PROFILE", "FULL_SRT")
        ).upper()
        self.radio_enabled = self.control_profile != "POINTING_ONLY"
        self.radio_center_frequency = config_dict["RADIO_CF"]
        self.radio_sample_frequency = config_dict["RADIO_SF"]
        self.radio_frequency_correction = config_dict["RADIO_FREQ_CORR"]
        self.radio_num_bins = config_dict["RADIO_NUM_BINS"]
        self.radio_integ_cycles = config_dict["RADIO_INTEG_CYCLES"]
        self.radio_autostart = config_dict["RADIO_AUTOSTART"]
        self.num_beamswitches = config_dict["NUM_BEAMSWITCHES"]
        self.beamwidth = config_dict["BEAMWIDTH"]
        self.observation_dwell_time = config_dict.get("OBSERVATION_DWELL_TIME", 5)
        self.scan_settle_time = config_dict.get("SCAN_SETTLE_TIME", 1.0)
        self.rotor_move_timeout = config_dict.get("ROTOR_MOVE_TIMEOUT", 180.0)
        deadband_mdeg = config_dict.get("TRACKING_COMMAND_DEADBAND_MDEG")
        if deadband_mdeg is None:
            # Backward compatibility for older configs that used degrees.
            deadband_deg = config_dict.get("TRACKING_COMMAND_DEADBAND_DEG", 0.15)
            deadband_mdeg = float(deadband_deg) * 1000.0
        self.tracking_command_deadband_deg = float(deadband_mdeg) / 1000.0
        self.pointing_error_threshold_mdeg = config_dict.get(
            "POINTING_ERROR_THRESHOLD_MDEG", 150.0
        )
        self.pointing_error_stable_cycles = int(
            config_dict.get("POINTING_ERROR_STABLE_CYCLES", 4)
        )
        self.temp_sys = config_dict["TSYS"]
        self.temp_cal = config_dict["TCAL"]
        self.save_dir = config_dict["SAVE_DIRECTORY"]

        # Generate Default Calibration Values
        # Values are Set Up so that Uncalibrated and Calibrated Spectra are the Same Values
        # Unless there is a pre-exisiting calibration from a previous run
        self.cal_values = [1.0 for _ in range(self.radio_num_bins)]
        self.cal_power = 1.0 / (self.temp_sys + self.temp_cal)
        calibration_path = Path(config_directory, "calibration.json")
        if calibration_path.is_file():
            with open(calibration_path, "r") as input_file:
                try:
                    cal_data = json.load(input_file)
                    # If Calibration is of a Different Size Than The Current FFT Size, Discard
                    if len(cal_data["cal_values"]) == self.radio_num_bins:
                        self.cal_values = cal_data["cal_values"]
                        self.cal_power = cal_data["cal_pwr"]
                except KeyError:
                    pass

        # Create Helper Object Which Tracks Celestial Objects
        self.ephemeris_tracker = EphemerisTracker(
            self.station["latitude"],
            self.station["longitude"],
            config_file=str(
                Path(config_directory, "sky_coords.csv").absolute()),
        )
        self.ephemeris_locations = self.ephemeris_tracker.get_all_azimuth_elevation()
        self.ephemeris_vlsr = self.ephemeris_tracker.get_all_vlsr()
        self.ephemeris_time_locs = self.ephemeris_tracker.get_all_azel_time()
        self.current_vlsr = 0.0
        self.ephemeris_cmd_location = None

        # Create Rotor Command Helper Object
        self.rotor = Rotor(
            self.motor_type,
            self.motor_port,
            self.motor_baudrate,
            self.az_limits,
            self.el_limits,
            motor_kwargs={
                "safe_mode": self.caltech6m_safe_mode,
            } if str(self.motor_type).upper() == "CALTECH6M" else None,
        )
        self.rotor_location = self.stow_location
        self.rotor_destination = self.stow_location
        self.rotor_offsets = (0.0, 0.0)
        self.rotor_cmd_location = tuple(
            map(add, self.rotor_destination, self.rotor_offsets)
        )

        # Set up radio task class references lazily so POINTING_ONLY does not
        # import GNU Radio/osmosdr dependencies at module import time.
        self.RadioProcessTask = None
        self.RadioSaveRawTask = None
        self.RadioCalibrateTask = None
        self.RadioSaveSpecRadTask = None
        self.RadioSaveSpecFitsTask = None
        self.radio_process_task = None
        if self.radio_enabled:
            try:
                self._load_radio_task_classes()
                assert self.RadioProcessTask is not None
                self.radio_process_task = self.RadioProcessTask(
                    num_bins=self.radio_num_bins,
                    num_integrations=self.radio_integ_cycles,
                )
            except ModuleNotFoundError as exc:
                # Permit dashboard/pointing use without GNU Radio stack installed.
                self.radio_enabled = False
                print(
                    "Radio dependencies unavailable "
                    f"({exc}); continuing in POINTING_ONLY behavior."
                )
        self.radio_queue = Queue()
        self.radio_save_task = None

        # Create Object for Keeping Track of What Commands Are Running or Have Failed
        self.current_queue_item = "None"
        self.command_queue = Queue()
        self.command_error_logs = []
        self.keep_running = True

        # List for data that will be plotted in the app
        self.n_point_data = []
        self.beam_switch_data = []
        self.pointing_error = None
        self.pointing_error_history = deque(maxlen=900)
        self.rotor_diagnostics = {}
        self.rotor_fsm_status = {}
        self.observation_events = deque(maxlen=1200)
        self.active_observation = None

    def log_message(self, message):
        """Writes Contents to a Logging List and Prints

        Parameters
        ----------
        message : str
            Message to Log and Print

        Returns
        -------
        None
        """
        self.command_error_logs.append((time(), message))
        print(message)

    def _record_observation_event(self, event_type, metadata=None):
        timestamp = time()
        event = {
            "time": timestamp,
            "iso_time": datetime.utcfromtimestamp(timestamp).isoformat() + "Z",
            "event": event_type,
            "metadata": metadata or {},
        }
        self.observation_events.append(event)

    def _begin_observation(self, metadata):
        # A new observation boundary implies previous observation ended by motion.
        self._end_active_observation("move_away")
        self._record_observation_event("settled_on_pointing", dict(metadata))
        self._record_observation_event("observation_begin", dict(metadata))
        self.active_observation = {
            "metadata": dict(metadata),
            "start_time": time(),
        }

    def _end_active_observation(self, reason):
        if self.active_observation is None:
            return
        end_meta = dict(self.active_observation.get("metadata", {}))
        end_meta["end_reason"] = reason
        self._record_observation_event("observation_end", end_meta)
        self.active_observation = None

    def get_observation_events_csv(self):
        """Export observation events as CSV string.
        
        Returns
        -------
        str
            CSV formatted string with headers and observation event data
        """
        import csv
        from io import StringIO
        
        output = StringIO()
        
        # Get list of observation events
        events = list(self.observation_events)
        if not events:
            return ""
        
        # Flatten the data for CSV
        rows = []
        fieldnames = [
            "iso_time", "event", "sequence", "object", "target_azel_az", 
            "target_azel_el", "end_reason", "point_index", "point_total"
        ]
        
        for event in events:
            row = {
                "iso_time": event.get("iso_time", ""),
                "event": event.get("event", ""),
                "sequence": event.get("metadata", {}).get("sequence", ""),
                "object": event.get("metadata", {}).get("object", ""),
                "target_azel_az": event.get("metadata", {}).get("target_azel", ("", ""))[0],
                "target_azel_el": event.get("metadata", {}).get("target_azel", ("", ""))[1],
                "end_reason": event.get("metadata", {}).get("end_reason", ""),
                "point_index": event.get("metadata", {}).get("point_index", ""),
                "point_total": event.get("metadata", {}).get("point_total", ""),
            }
            rows.append(row)
        
        # Write CSV
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
        
        return output.getvalue()

    def _radio_put(self, method, value):
        """Queues radio updates only when radio integration is enabled."""
        if self.radio_enabled:
            self.radio_queue.put((method, value))

    def _wait_for_rotor_target(self, settle_time=0.0):
        """Wait for rotor to reach the current command location and settle."""
        if not self.rotor.is_motion_allowed():
            self.log_message("Motion disabled (safe mode): skipping wait for rotor target")
            return False
        timeout = max(0.1, float(self.rotor_move_timeout))
        start_time = time()

        stable_needed = max(1, int(self.pointing_error_stable_cycles))
        stable_count = 0

        while True:
            reached_by_position = azel_within_range(
                self.rotor_location, self.rotor_cmd_location
            )
            point_err = self.rotor.get_pointing_error()
            reached_by_error = False
            if point_err is not None:
                azerr_mdeg, elerr_mdeg = point_err
                threshold = abs(float(self.pointing_error_threshold_mdeg))
                if abs(azerr_mdeg) <= threshold and abs(elerr_mdeg) <= threshold:
                    stable_count += 1
                else:
                    stable_count = 0
                reached_by_error = stable_count >= stable_needed

            if reached_by_error or reached_by_position:
                break

            if time() - start_time > timeout:
                self.log_message(
                    f"Timed out waiting for rotor to reach {self.rotor_cmd_location}"
                )
                return False
            sleep(0.1)

        settle_seconds = max(0.0, float(settle_time))
        if settle_seconds > 0:
            sleep(settle_seconds)
        return True

    def _load_radio_task_classes(self):
        """Import radio task classes only when radio features are enabled."""
        if self.RadioProcessTask is not None:
            return
        from .radio_control.radio_task_starter import (
            RadioProcessTask,
            RadioSaveRawTask,
            RadioCalibrateTask,
            RadioSaveSpecRadTask,
            RadioSaveSpecFitsTask,
        )

        self.RadioProcessTask = RadioProcessTask
        self.RadioSaveRawTask = RadioSaveRawTask
        self.RadioCalibrateTask = RadioCalibrateTask
        self.RadioSaveSpecRadTask = RadioSaveSpecRadTask
        self.RadioSaveSpecFitsTask = RadioSaveSpecFitsTask

    def n_point_scan(self, object_id):
        """Runs an N-Point (25) Scan About an Object

        Parameters
        ----------
        object_id : str
            Name of the Object to Perform N-Point Scan About

        Returns
        -------
        None
        """
        self.ephemeris_cmd_location = None
        self._radio_put("soutrack", object_id)
        # Send vlsr to radio queue
        cur_vlsr = self.ephemeris_vlsr[object_id]
        self._radio_put("vlsr", float(cur_vlsr))
        self.current_vlsr = cur_vlsr
        N_pnt_default = 25
        rotor_loc = []
        pwr_list = []
        #
        scan_center = self.ephemeris_locations[object_id]
        np_sides = [5, 5]
        for scan in range(N_pnt_default):
            self._end_active_observation("move_away")
            scan_center = self.ephemeris_locations[object_id]
            self.log_message(
                "{0} of {1} point scan.".format(scan, N_pnt_default))
            i = (scan // 5) - 2
            j = (scan % 5) - 2
            el_dif = i * self.beamwidth * 0.5
            az_dif_scalar = np.cos((scan_center[1] + el_dif) * np.pi / 180.0)
            # Avoid issues where you get close to the zenith
            if np.abs(az_dif_scalar) < 1e-4:
                az_dif = 0
            else:
                az_dif = j * self.beamwidth * 0.5 / az_dif_scalar

            new_rotor_offsets = (az_dif, el_dif)

            if self.rotor.angles_within_bounds(*scan_center):
                self.rotor_destination = scan_center
                if not self.point_at_offset(
                    *new_rotor_offsets,
                    observation_context={
                        "sequence": "n_point",
                        "object": object_id,
                        "point_index": int(scan + 1),
                        "point_total": int(N_pnt_default),
                    },
                ):
                    continue
            rotor_loc.append(self.rotor_location)
            
            # Only collect spectrum if radio is enabled
            sleep(max(0.0, float(self.observation_dwell_time)))
            if self.radio_enabled:
                raw_spec = get_spectrum(port=5561)
                if raw_spec is None or len(raw_spec) == 0:
                    continue
                p = np.sum(raw_spec)
                a = len(raw_spec)
                pwr = (self.temp_sys + self.temp_cal) * p / (a * self.cal_power)
                pwr_list.append(pwr)
        self._end_active_observation("sequence_complete")
                
        maxdiff = (az_dif, el_dif)
        self.n_point_data = [scan_center, maxdiff,
                             rotor_loc, pwr_list, np_sides]

        # add code to collect spectrum data.
        self.rotor_offsets = (0.0, 0.0)
        self.ephemeris_cmd_location = object_id

    def beam_switch(self, object_id):
        """Swings Antenna Across Object

        Parameters
        ----------
        object_id : str
            Name of the Object to Perform Beam-Switch About

        Returns
        -------
        None
        """
        self.ephemeris_cmd_location = None
        self._radio_put("soutrack", object_id)
        # Send vlsr to radio queue
        cur_vlsr = self.ephemeris_vlsr[object_id]
        self._radio_put("vlsr", float(cur_vlsr))
        self.current_vlsr = cur_vlsr
        rotor_loc = []
        pwr_list = []
        for j in range(0, 3 * self.num_beamswitches):
            self._end_active_observation("move_away")
            new_rotor_destination = self.ephemeris_locations[object_id]
            self._radio_put("beam_switch", j + 1)
            az_dif_scalar = np.cos(new_rotor_destination[1] * np.pi / 180.0)
            az_dif = (j % 3 - 1) * self.beamwidth / az_dif_scalar
            new_rotor_offsets = (az_dif, 0)
            if self.rotor.angles_within_bounds(*new_rotor_destination):
                self.rotor_destination = new_rotor_destination
                if not self.point_at_offset(
                    *new_rotor_offsets,
                    observation_context={
                        "sequence": "beam_switch",
                        "object": object_id,
                        "point_index": int(j + 1),
                        "point_total": int(3 * self.num_beamswitches),
                    },
                ):
                    continue
            rotor_loc.append(self.rotor_location)
            
            # Only collect spectrum if radio is enabled
            sleep(max(0.0, float(self.observation_dwell_time)))
            if self.radio_enabled:
                raw_spec = get_spectrum(port=5561)
                if raw_spec is None or len(raw_spec) == 0:
                    continue
                p = np.sum(raw_spec)
                a = len(raw_spec)
                pwr = (self.temp_sys + self.temp_cal) * p / (a * self.cal_power)
                pwr_list.append(pwr)
        self._end_active_observation("sequence_complete")
                
        self.rotor_offsets = (0.0, 0.0)
        self._radio_put("beam_switch", 0)
        self.ephemeris_cmd_location = object_id
        self.beam_switch_data = [rotor_loc, pwr_list]

    def point_at_object(self, object_id):
        """Points Antenna Directly at Object, and Sets Up Tracking to Follow it

        Parameters
        ----------
        object_id : str
            Name of Object to Point at and Track

        Returns
        -------
        None
        """
        self.rotor_offsets = (0.0, 0.0)
        self._radio_put("soutrack", object_id)
        # Send vlsr to radio queue
        cur_vlsr = self.ephemeris_vlsr[object_id]
        self._radio_put("vlsr", float(cur_vlsr))
        self.current_vlsr = cur_vlsr
        new_rotor_cmd_location = self.ephemeris_locations[object_id]
        self._end_active_observation("move_away")
        if self.rotor.angles_within_bounds(*new_rotor_cmd_location):
            self.ephemeris_cmd_location = object_id
            self.rotor_destination = new_rotor_cmd_location
            self.rotor_cmd_location = new_rotor_cmd_location
            if self._wait_for_rotor_target():
                self._begin_observation(
                    {
                        "sequence": "point_object",
                        "object": object_id,
                        "target_azel": [
                            float(self.rotor_cmd_location[0]),
                            float(self.rotor_cmd_location[1]),
                        ],
                    }
                )
        else:
            self.log_message(f"Object {object_id} Not in Motor Bounds")
            self.ephemeris_cmd_location = None

    def point_at_azel(self, az, el):
        """Points Antenna at a Specific Azimuth and Elevation

        Parameters
        ----------
        az : float
            Azimuth, in degrees, to turn antenna towards
        el : float
            Elevation, in degrees, to point antenna upwards at

        Returns
        -------
        None
        """
        # cur_vlsr = self.ephemeris_tracker.calculate_vlsr_azel((az,el))
        # self.radio_queue.put(("vlsr",cur_vlsr))
        # self.current_vlsr = cur_vlsr
        self.ephemeris_cmd_location = None
        self.rotor_offsets = (0.0, 0.0)
        # Send az and el angles to sources track for the radio
        self._radio_put("soutrack", f"azel_{az}_{el}")
        # Send vlsr to radio queue
        cur_vlsr = self.ephemeris_tracker.calculate_vlsr_azel((az, el))
        self.current_vlsr = cur_vlsr
        self._radio_put("vlsr", float(cur_vlsr))

        new_rotor_destination = (az, el)
        new_rotor_cmd_location = new_rotor_destination
        self._end_active_observation("move_away")
        if self.rotor.angles_within_bounds(*new_rotor_cmd_location):
            self.rotor_destination = new_rotor_destination
            self.rotor_cmd_location = new_rotor_cmd_location
            if self._wait_for_rotor_target():
                self._begin_observation(
                    {
                        "sequence": "point_azel",
                        "target_azel": [
                            float(self.rotor_cmd_location[0]),
                            float(self.rotor_cmd_location[1]),
                        ],
                    }
                )
        else:
            self.log_message(
                f"Object at {new_rotor_cmd_location} Not in Motor Bounds")

    def point_at_offset(self, az_off, el_off, observation_context=None):
        """From the Current Object or Position Pointed At, Move to an Offset of That Location

        Parameters
        ----------
        az_off : float
            Number of Degrees in Azimuth Offset
        el_off : float
            Number of Degrees in Elevation Offset

        Returns
        -------
        bool
            True if destination was reached, False otherwise.
        """
        new_rotor_offsets = (az_off, el_off)
        new_rotor_cmd_location = tuple(
            map(add, self.rotor_destination, new_rotor_offsets)
        )
        self._end_active_observation("move_away")
        if self.rotor.angles_within_bounds(*new_rotor_cmd_location):
            self.rotor_offsets = new_rotor_offsets
            self.rotor_cmd_location = new_rotor_cmd_location
            if self._wait_for_rotor_target():
                self._begin_observation(
                    {
                        "sequence": "offset_point",
                        "target_azel": [
                            float(self.rotor_cmd_location[0]),
                            float(self.rotor_cmd_location[1]),
                        ],
                        "offset": [float(az_off), float(el_off)],
                        **(observation_context or {}),
                    }
                )
                return True
            return False
        else:
            self.log_message(f"Offset {new_rotor_offsets} Out of Bounds")
            return False

    def stow(self):
        """Moves the Antenna Back to Its Stow Location

        Returns
        -------
        None
        """
        self.ephemeris_cmd_location = None
        self._radio_put("soutrack", "at_stow")
        self._end_active_observation("move_away")
        self.rotor_offsets = (0.0, 0.0)
        self.rotor_destination = self.stow_location
        self.rotor_cmd_location = self.stow_location
        self._wait_for_rotor_target()

    def calibrate(self):
        """Runs Calibration Processing and Pushes New Values to Processing Script

        Returns
        -------
        None
        """
        if not self.radio_enabled:
            self.log_message(
                "Radio calibration is unavailable in POINTING_ONLY profile"
            )
            return

        sleep(
            self.radio_num_bins * self.radio_integ_cycles / self.radio_sample_frequency
        )
        self._load_radio_task_classes()
        assert self.RadioCalibrateTask is not None
        radio_cal_task = self.RadioCalibrateTask(
            self.radio_num_bins,
            self.config_directory,
        )
        radio_cal_task.start()
        radio_cal_task.join(30)
        path = Path(self.config_directory, "calibration.json")
        with open(path, "r") as input_file:
            cal_data = json.load(input_file)
            self.cal_values = cal_data["cal_values"]
            self.cal_power = cal_data["cal_pwr"]
        self._radio_put("cal_pwr", self.cal_power)
        self._radio_put("cal_values", self.cal_values)
        self.log_message("Calibration Done")

    def calibrate_encoders(self):
        """Runs motor encoder calibration when supported by motor backend."""
        self._end_active_observation("encoder_calibration")
        if not hasattr(self.rotor.motor, "calibrate"):
            self.log_message("Encoder calibration is unavailable for this motor backend")
            return
        self.log_message("Starting 6m encoder calibration")
        self.rotor.motor.calibrate()
        cal_sts = getattr(self.rotor.motor, "CalSts", "Unknown")
        self.log_message(f"Encoder calibration complete: {cal_sts}")

    def start_recording(self, name):
        """Starts Recording Data

        Parameters
        ----------
        name : str
            Name of the File to Be Recorded

        Returns
        -------
        None
        """
        if not self.radio_enabled:
            self.log_message("Recording is unavailable in POINTING_ONLY profile")
            return

        if self.radio_save_task is None:
            self._load_radio_task_classes()
            if name is None:
                assert self.RadioSaveRawTask is not None
                self.radio_save_task = self.RadioSaveRawTask(
                    self.radio_sample_frequency, self.save_dir, name
                )
            elif name.endswith(".rad"):
                name = None if name == "*.rad" else name
                assert self.RadioSaveSpecRadTask is not None
                self.radio_save_task = self.RadioSaveSpecRadTask(
                    self.radio_sample_frequency,
                    self.radio_num_bins,
                    self.save_dir,
                    name,
                )
            elif name.endswith(".fits"):
                name = None if name == "*.fits" else name
                assert self.RadioSaveSpecFitsTask is not None
                self.radio_save_task = self.RadioSaveSpecFitsTask(
                    self.radio_sample_frequency,
                    self.radio_num_bins,
                    self.save_dir,
                    name,
                )
            else:
                assert self.RadioSaveRawTask is not None
                self.radio_save_task = self.RadioSaveRawTask(
                    self.radio_sample_frequency, self.save_dir, name
                )
            self.radio_save_task.start()
        else:
            self.log_message("Cannot Start Recording - Already Recording")

    def stop_recording(self):
        """Stops Any Current Recording, if Running

        Returns
        -------
        None
        """
        if not self.radio_enabled:
            self.log_message("Recording is unavailable in POINTING_ONLY profile")
            return

        if self.radio_save_task is not None:
            self.radio_save_task.terminate()
            self.radio_save_task = None

    def set_freq(self, frequency):
        """Set the Frequency of the Processing Script

        Parameters
        ----------
        frequency : float
            Center Frequency, in Hz, to Set SDR to

        Returns
        -------
        None
        """
        self.radio_center_frequency = frequency  # Set Local Value
        self._radio_put(
            "freq", self.radio_center_frequency + self.radio_frequency_correction
        )  # Push Update to GNU Radio

    def set_coords(self, lat, long, config_directory="config/sky_coords.csv", name=None):
        """Set the lat/long coordinates of observer location

        Parameters
        ----------
        lat : float
            Observer Latitude, in degs, to Set SDR to

        long: float
            Observer Longitude, in degs, to Set SDR to

        name: string
            Observer location name, None if not given

        Returns
        -------
        None
        """
        self.station = {"latitude": lat,
                        "longitude": long,
                        "name": name}
        self.ephemeris_tracker = EphemerisTracker(
            self.station["latitude"],
            self.station["longitude"],
            # config_file=str(
            #     Path(config_directory, "sky_coords.csv").absolute()),
        )
        # self.radio_queue.put((""))

    def set_samp_rate(self, samp_rate):
        """Set the Sample Rate of the Processing Script

        Note that this stops any currently running raw saving tasks

        Parameters
        ----------
        samp_rate : float
            Sample Rate for the SDR in Hz

        Returns
        -------
        None
        """
        if self.radio_save_task is not None:
            self.radio_save_task.terminate()
        self.radio_sample_frequency = samp_rate
        self._radio_put("samp_rate", self.radio_sample_frequency)

    def quit(self):
        """Stops the Daemon Process

        Returns
        -------
        None
        """
        self.keep_running = False
        self._radio_put("is_running", self.keep_running)

    def find_object_location(self, name):
        """Get azel location of given object and sets as rotor location. 

        Returns
        -------
        None
        """
        if name in self.ephemeris_tracker.az_el_dict:
            az, el = self.ephemeris_tracker.az_el_dict[name][0], self.ephemeris_tracker.az_el_dict[name][1]
            self.log_message(f"here {az,el}")

        self.rotor_location = (az, el)

        self.rotor_offsets = (0.0, 0.0)

        new_rotor_cmd_location = (az, el)
        self._end_active_observation("move_away")
        if self.rotor.angles_within_bounds(*new_rotor_cmd_location):
            self.ephemeris_cmd_location = name
            self.rotor_destination = new_rotor_cmd_location
            self.rotor_cmd_location = new_rotor_cmd_location
            if self._wait_for_rotor_target():
                self._begin_observation(
                    {
                        "sequence": "find_object_location",
                        "object": name,
                        "target_azel": [
                            float(self.rotor_cmd_location[0]),
                            float(self.rotor_cmd_location[1]),
                        ],
                    }
                )
        else:
            self.log_message(f"Object {name} Not in Motor Bounds")
            self.ephemeris_cmd_location = None

    def update_ephemeris_location(self):
        """Periodically Updates Object Locations for Tracking Sky Objects

        Is Operated as an Infinite Looping Thread Function
        
        NOTE: Continuous tracking position updates (ephemeris-driven) do NOT trigger
        observation_end events. These are background position adjustments to follow
        objects across the sky. Observations only end when an explicit motion command
        is issued (point_at_object, point_at_azel, point_at_offset, stow, or calibration).

        Returns
        -------
        None
        """
        last_updated_time = None
        while True:
            if last_updated_time is None or time() - last_updated_time > 10:
                last_updated_time = time()
                self.ephemeris_tracker.update_all_az_el()
            self.ephemeris_locations = (
                self.ephemeris_tracker.get_all_azimuth_elevation()
            )
            self.ephemeris_vlsr = self.ephemeris_tracker.get_all_vlsr()
            self.ephemeris_time_locs = (
                self.ephemeris_tracker.get_all_azel_time()
            )
            if self.ephemeris_cmd_location is not None:
                # Update position command location based on ephemeris tracking
                # This maintains pointing on the target as it moves across the sky
                # Update_rotor_status() will issue the actual motion command if needed
                new_rotor_destination = self.ephemeris_locations[
                    self.ephemeris_cmd_location
                ]
                self.current_vlsr = self.ephemeris_vlsr[self.ephemeris_cmd_location]
                new_rotor_cmd_location = tuple(
                    map(add, new_rotor_destination, self.rotor_offsets)
                )
                if self.rotor.angles_within_bounds(
                    *new_rotor_destination
                ) and self.rotor.angles_within_bounds(*new_rotor_cmd_location):
                    self.rotor_destination = new_rotor_destination
                    self.rotor_cmd_location = new_rotor_cmd_location
                else:
                    self.log_message(
                        f"Object {self.ephemeris_cmd_location} moved out of motor bounds"
                    )
                    self.ephemeris_cmd_location = None
            sleep(1)

    def update_rotor_status(self):
        """Periodically Sets Rotor Azimuth and Elevation and Fetches New Antenna Position

        Is Operated as an Infinite Looping Thread Function

        Returns
        -------
        None
        """
        while True:
            try:
                current_rotor_cmd_location = self.rotor_cmd_location
                if not azel_within_range(
                    self.rotor_location,
                    current_rotor_cmd_location,
                    bounds=(
                        float(self.tracking_command_deadband_deg),
                        float(self.tracking_command_deadband_deg),
                    ),
                ):
                    if self.rotor.is_motion_allowed():
                        self.rotor.set_azimuth_elevation(*current_rotor_cmd_location)

                past_rotor_location = self.rotor_location
                self.rotor_location = self.rotor.get_azimuth_elevation()
                self.pointing_error = self.rotor.get_pointing_error()
                self.rotor_diagnostics = self.rotor.get_diagnostics()
                self.rotor_fsm_status = self.rotor.get_fsm_status()
                if self.pointing_error is not None:
                    self.pointing_error_history.append(
                        {
                            "time": time(),
                            "azerr_mdeg": float(self.pointing_error[0]),
                            "elerr_mdeg": float(self.pointing_error[1]),
                        }
                    )
                if not self.rotor_location == past_rotor_location:
                    g_lat, g_lon = self.ephemeris_tracker.convert_to_gal_coord(
                        self.rotor_location
                    )
                    self._radio_put("motor_az", float(self.rotor_location[0]))
                    self._radio_put("motor_el", float(self.rotor_location[1]))
                    self._radio_put("glat", g_lat)
                    self._radio_put("glon", g_lon)

                sleep(0.1)
            except AssertionError as e:
                self.log_message(str(e))
            except ValueError as e:
                self.log_message(str(e))

    def update_status(self):
        """Periodically Publishes Daemon Status for Dashboard.

        Is Operated as an Infinite Looping Thread Function

        Returns
        -------
        None
        """
        context = zmq.Context()
        
        status_port = 5555
        status_socket = context.socket(zmq.PUB)
        status_socket.bind("tcp://127.0.0.1:%s" % status_port)
        logging.warning("Status socket bound on localhost:%s", status_port)

        def _json_default(obj):
            """Best-effort conversion for status payload serialization."""
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, (np.floating, np.integer, np.bool_)):
                return obj.item()
            if isinstance(obj, bytes):
                return obj.decode("utf-8", errors="replace")
            return str(obj)
        
        while True:
            try:
                status = {
                    "beam_width": self.beamwidth,
                    "location": self.station,
                    "motor_azel": self.rotor_location,
                    "motor_cmd_azel": self.rotor_cmd_location,
                    "vlsr": self.current_vlsr,
                    "object_locs": self.ephemeris_locations,
                    "object_time_locs": self.ephemeris_time_locs,
                    "az_limits": self.az_limits,
                    "el_limits": self.el_limits,
                    "stow_loc": self.stow_location,
                    "cal_loc": self.cal_location,
                    "horizon_points": self.horizon_points,
                    "center_frequency": self.radio_center_frequency,
                    "frequency_correction": self.radio_frequency_correction,
                    "bandwidth": self.radio_sample_frequency,
                    "motor_offsets": self.rotor_offsets,
                    "queued_item": self.current_queue_item,
                    "queue_size": self.command_queue.qsize(),
                    "emergency_contact": self.contact,
                    "error_logs": self.command_error_logs,
                    "temp_cal": self.temp_cal,
                    "temp_sys": self.temp_sys,
                    "cal_power": self.cal_power,
                    "n_point_data": self.n_point_data,
                    "beam_switch_data": self.beam_switch_data,
                    "pointing_error": self.pointing_error,
                    "pointing_error_history": list(self.pointing_error_history),
                    "rotor_diagnostics": self.rotor_diagnostics,
                    "rotor_fsm_status": self.rotor_fsm_status,
                    "observation_events": list(self.observation_events)[-80:],
                    "time": time(),
                }
                try:
                    status["serial_communications"] = (
                        self.rotor.get_recent_serial_communications(limit=12)
                    )
                except Exception:
                    status["serial_communications"] = []

                try:
                    status["command_history"] = self.rotor.get_recent_command_history(
                        limit=12
                    )
                except Exception:
                    status["command_history"] = []

                serialized = json.dumps(status, default=_json_default)
                status_socket.send_string(serialized)
            except Exception as e:
                logging.exception("Status publish error: %s", str(e))
            sleep(0.5)

    def update_radio_settings(self):
        """Coordinates Sending XMLRPC Commands to the GNU Radio Script

        Is Operated as an Infinite Looping Thread Function

        Returns
        -------
        None
        """
        rpc_server = ServerProxy("http://localhost:5557/")
        while True:
            method, value = self.radio_queue.get()
            call = getattr(rpc_server, f"set_{method}")
            call(value)
            sleep(0.1)

    def update_command_queue(self):
        """Waits for New Commands Coming in Over ZMQ PUSH/PULL with PLAIN auth

        Is Operated as an Infinite Looping Thread Function

        Returns
        -------
        None
        """
        context = zmq.Context()
        
        command_port = 5556
        command_socket = context.socket(zmq.PULL)
        command_socket.bind("tcp://127.0.0.1:%s" % command_port)
        logging.warning("Command socket bound on localhost:%s", command_port)
        
        while True:
            try:
                cmd = command_socket.recv_string()
                self.command_queue.put(cmd)
            except zmq.error.ZMQError as e:
                logging.error("Command socket error: %s", str(e))

    def srt_daemon_main(self):
        """Starts and Processes Commands for the SRT

        Returns
        -------
        None
        """

        # Create Infinite Looping Threads
        ephemeris_tracker_thread = Thread(
            target=self.update_ephemeris_location, daemon=True
        )
        rotor_pointing_thread = Thread(
            target=self.update_rotor_status, daemon=True)
        command_queueing_thread = Thread(
            target=self.update_command_queue, daemon=True)
        status_thread = Thread(target=self.update_status, daemon=True)
        radio_thread = Thread(target=self.update_radio_settings, daemon=True)

        # If the GNU Radio Script Should be Running, Start It
        if self.radio_enabled and self.radio_autostart:
            try:
                assert self.radio_process_task is not None
                self.radio_process_task.start()
            except RuntimeError as e:
                self.log_message(str(e))
            sleep(5)

        # Send Settings to the GNU Radio Script
        radio_params = {
            "Frequency": (
                "freq",
                self.radio_center_frequency + self.radio_frequency_correction,
            ),
            "Sample Rate": ("samp_rate", self.radio_sample_frequency),
            "Motor Azimuth": ("motor_az", self.rotor_location[0]),
            "Motor Elevation": ("motor_el", self.rotor_location[1]),
            "Motor GalLat": (
                "glat",
                self.ephemeris_tracker.convert_to_gal_coord(
                    self.rotor_location)[0],
            ),
            "Motor GalLon": (
                "glon",
                self.ephemeris_tracker.convert_to_gal_coord(
                    self.rotor_location)[1],
            ),
            "Object Tracking": ("soutrack", "at_stow"),
            "System Temp": ("tsys", self.temp_sys),
            "Calibration Temp": ("tcal", self.temp_cal),
            "Calibration Power": ("cal_pwr", self.cal_power),
            "Calibration Values": ("cal_values", self.cal_values),
            "Is Running": ("is_running", True),
        }
        if self.radio_enabled:
            for name in radio_params:
                self.log_message(f"Setting {name}")
                self.radio_queue.put(radio_params[name])

        # Start Infinite Looping Update Threads
        ephemeris_tracker_thread.start()
        rotor_pointing_thread.start()
        command_queueing_thread.start()
        status_thread.start()
        if self.radio_enabled:
            radio_thread.start()

        while self.keep_running:
            try:
                # Await Command for the SRT
                self.current_queue_item = "None"
                command = self.command_queue.get()
                self.log_message(f"Running Command '{command}'")
                self.current_queue_item = command
                if len(command) < 2 or command[0] == "*":
                    continue
                elif command[0] == ":":
                    command = command[1:].strip()
                command_parts = command.split(" ")
                command_parts = [x for x in command_parts if x]
                command_name = command_parts[0].lower()

                # Handle Stop All (SPA) command - Emergency stop for motors
                if command_name == "spa":
                    self.log_message("SPA command received - Stopping all rotor motors")
                    try:
                        self.rotor.motor.stop()
                        self.log_message("Rotor motors stopped successfully")
                    except Exception as e:
                        self.log_message(f"Error stopping rotor: {str(e)}")
                    continue  # Continue processing commands, don't shut down

                # If Command Starts With a Valid Object Name
                if command_parts[0] in self.ephemeris_locations:
                    if command_parts[-1] == "n":  # N-Point Scan About Object
                        self.n_point_scan(object_id=command_parts[0])
                    elif command_parts[-1] == "b":  # Beam-Switch Away From Object
                        self.beam_switch(object_id=command_parts[0])
                    else:  # Point Directly At Object
                        self.point_at_object(object_id=command_parts[0])
                elif command_name == "stow":
                    self.stow()
                elif command_name == "cal":
                    self.point_at_azel(*self.cal_location)
                elif command_name == "calibrate":
                    self.calibrate()
                elif command_name == "calibrate_encoders":
                    self.calibrate_encoders()
                elif command_name == "quit":
                    self.quit()
                elif command_name == "record":
                    self.start_recording(
                        name=(None if len(command_parts)
                              <= 1 else command_parts[1])
                    )
                elif command_name == "roff":
                    self.stop_recording()
                elif command_name == "freq":
                    self.set_freq(frequency=float(
                        command_parts[1]) * pow(10, 6))
                elif command_name == "samp":
                    self.set_samp_rate(samp_rate=float(
                        command_parts[1]) * pow(10, 6))
                elif command_name == "coords":
                    self.set_coords(
                        float(command_parts[1]), float(command_parts[2]))
                elif command_name == "object":
                    if command_parts[-1] in self.ephemeris_locations:
                        self.find_object_location(command_parts[-1])
                elif command_name == "obj_coords":
                    self.rotor_location = (
                        float(command_parts[1]), float(command_parts[2]))
                elif command_name == "azel":
                    self.point_at_azel(
                        float(command_parts[1]),
                        float(command_parts[2]),
                    )
                elif command_name == "offset":
                    self.point_at_offset(
                        float(command_parts[1]), float(command_parts[2])
                    )
                elif (
                    command_name.isnumeric()
                ):  # If Command is a Number, Sleep that Long
                    sleep(float(command_name))
                elif command_name == "wait":
                    sleep(float(command_parts[1]))
                # Wait Until Next Time H:M:S
                elif command_name.split(":")[0] == "lst":
                    time_string = command_name.replace("LST:", "")
                    time_val = datetime.strptime(time_string, "%H:%M:%S")
                    while time_val < datetime.utcfromtimestamp(time()):
                        time_val += timedelta(days=1)
                    time_delta = (
                        time_val - datetime.utcfromtimestamp(time())
                    ).total_seconds()
                    sleep(time_delta)
                elif len(command_name.split(":")) == 5:  # Wait Until Y:D:H:M:S
                    time_val = datetime.strptime(
                        command_name, "%Y:%j:%H:%M:%S")
                    time_delta = (
                        time_val - datetime.utcfromtimestamp(time())
                    ).total_seconds()
                    sleep(time_delta)
                else:
                    self.log_message(f"Command Not Identified '{command}'")
                self.command_queue.task_done()
            except IndexError as e:
                self.log_message(str(e))
            except ValueError as e:
                self.log_message(str(e))
            except ConnectionRefusedError as e:
                self.log_message(str(e))

        # On End, Return to Stow and End Recordings
        self.stop_recording()
        # self.stow()
        if self.radio_enabled and self.radio_autostart:
            assert self.radio_process_task is not None
            sleep(1)
            self.radio_process_task.terminate()
