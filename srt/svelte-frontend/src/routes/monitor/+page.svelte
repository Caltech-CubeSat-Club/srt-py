<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import { telescopeState, liveTelescopeData } from '$lib/stores/telescope_state';
	import type { DaemonStatus } from '$lib/generated/types';
	import { Canvas } from '@threlte/core'
	import type { CameraControlsRef } from '@threlte/extras';
	import ControlPanel from '$lib/ControlPanel.svelte';
	import { cursorAzEl } from '$lib/stores/ui';

	let { data } = $props();

	let controls = $state.raw<CameraControlsRef>()

	let connectionState: 'connecting' | 'open' | 'closed' = $state('connecting');
	let socket: WebSocket | null = null;
	let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
	let destroyed = false;


	let time = $state(liveTelescopeData.time);
	let telescopeAzEl = $state([0, 0]);
	$effect(() => {
		const interval = setInterval(() => {
		time = liveTelescopeData.time;
		telescopeAzEl = [liveTelescopeData.rotor.az ?? 0, liveTelescopeData.rotor.el ?? 0];
		}, 500);
		return () => clearInterval(interval);
	});

	let cursorAzElValue = $state<[number, number]>([0, 0]);
	cursorAzEl.subscribe(([az, el]) => {
		cursorAzElValue = [az, el];
	});

	function connect(token: string) {
		connectionState = 'connecting';
		const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
		socket = new WebSocket(`${protocol}://${window.location.host}/ws/status?token=${encodeURIComponent(token)}`);

		socket.onopen = () => (connectionState = 'open');

		socket.onmessage = (event) => {
			const status: DaemonStatus = JSON.parse(event.data);

			telescopeState.set({
				fsm_state: status.rotor?.fsm_state,
				cal_sts: status.rotor?.cal_sts,
				loop_mode: status.rotor?.loop_mode,
				error_logs: status.error_logs,
				observation_events: status.observation_events,
				serial_communications: status.serial_communications,
				command_history: status.command_history,
				location: status.location,
				beamwidth: status.beam_width
			});

			// Mutate in place — no store, no reactivity, Threlte reads this
			// directly on its own render loop.
			Object.assign(liveTelescopeData.rotor, status.rotor);
			liveTelescopeData.spectrum = status.spectrum ?? null; // null when the instrument isn't connected
			liveTelescopeData.time = status.time ?? Date.now() / 1000; // fallback to local time if not provided
			Object.assign(liveTelescopeData.object_locs, status.object_locs);
		 };

		socket.onclose = () => {
			connectionState = 'closed';
			// Reconnect after a short delay — covers the daemon/dashboard
			// process restarting, a brief network blip, etc. Skipped if
			// the component has already been torn down (onDestroy), so
			// navigating away doesn't leave a zombie retry loop running.
			if (!destroyed) {
				reconnectTimer = setTimeout(() => connect(token), 1000);
			}
		};
	}

	onMount(() => {
		connect(data.token);
	});

	onDestroy(() => {
		destroyed = true;
		if (reconnectTimer) clearTimeout(reconnectTimer);
		socket?.close();
	});
</script>

<div class="p-4">
	<h1>Telescope Monitor</h1>
	<p>Connection: {connectionState}</p>
	<p>Time: {new Date(time * 1000).toISOString()}</p>
	<p>Cursor Az/El: {cursorAzElValue[0].toFixed(2)}°, {cursorAzElValue[1].toFixed(2)}°</p>
	<p>Rotor Az/El: {telescopeAzEl[0].toFixed(2)}°, {telescopeAzEl[1].toFixed(2)}°</p>

	<div class="border-2 border-gray-700 rounded-lg h-[80vh]">
		<Canvas>
			<ControlPanel bind:controls />
		</Canvas>
	</div>
</div>