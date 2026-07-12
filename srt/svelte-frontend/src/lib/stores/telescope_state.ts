import { writable } from 'svelte/store';
import type {
	DriverState,
	CalSts,
	LoopMode,
    ErrorLogs,
	ObservationEvent,
	SerialCommunication,
	CommandHistoryEntry,
    RotorState,
    SpectrumFrame,
    Location,
    ObjectLocs,
    Beamwidth
} from '$lib/generated/types';

export interface TelescopeState {
	fsm_state?: DriverState;
	cal_sts?: CalSts;
	loop_mode?: LoopMode;
	error_logs?: ErrorLogs;
	observation_events?: ObservationEvent[];
	serial_communications?: SerialCommunication[];
	command_history?: CommandHistoryEntry[];
    location?: Location;
    beamwidth?: Beamwidth;
}

export const telescopeState = writable<TelescopeState>({});


// High-frequency / real-time fields — read directly by Threlte's
// render loop (useTask, etc.), NOT through Svelte's reactivity system.
// Mutated in place by the WebSocket handler on every tick; Threlte
// reads whatever the current values are on its own render cadence,
// rather than re-rendering in lockstep with each WebSocket message.
export interface LiveTelescopeData {
    rotor: RotorState;
    spectrum: SpectrumFrame | null;
    time: number;
    object_locs: ObjectLocs;
}

export const liveTelescopeData: LiveTelescopeData = {
	rotor: {
		  az: undefined,
          el: undefined,
          az_err: undefined,
          el_err: undefined,
          az_cmd: undefined,
          el_cmd: undefined,
          fsm_state: "disconnected",
          last_transition: undefined,
          last_error: undefined,
          retry_count: undefined,
          safe_mode: undefined,
          cal_sts: undefined,
          loop_mode: undefined,
          az_brake: undefined,
          el_brake: undefined,
          estop: undefined,
          sim_mode: undefined,
          el_up_pre: undefined,
          el_dn_pre: undefined,
          el_up_fin: undefined,
          el_dn_fin: undefined,
          az_cw_pre: undefined,
          az_ccw_pre: undefined,
          az_cw_fin: undefined,
          az_ccw_fin: undefined,
          az_lt_180: undefined,
          amp_currents: undefined,
          lpr: undefined,
          last_poll_time: undefined,
          last_command_time: undefined
	} as RotorState,
	spectrum: null,
    time: 0,
    object_locs: {} as ObjectLocs
};
