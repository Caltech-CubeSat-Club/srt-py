<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import { telescopeState, liveTelescopeData } from '$lib/stores/telescope_state';
	import type { DaemonStatus } from '$lib/generated/types';
	import { Canvas } from '@threlte/core'
	import type { CameraControlsRef } from '@threlte/extras';
	import ControlPanel from '$lib/ControlPanel.svelte';

	let { data } = $props();

	let controls = $state.raw<CameraControlsRef>()

	let connectionState: 'connecting' | 'open' | 'closed' = $state('connecting');
	let socket: WebSocket | null = null;
	let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
	let destroyed = false;

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
				location: status.location
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

<h1>Telescope Monitor</h1>
<p>Connection: {connectionState}</p>

<div class="m-2 border-2 border-gray-700 rounded-lg h-[80vh]">
<Canvas>
	<ControlPanel bind:controls />
</Canvas>
</div>