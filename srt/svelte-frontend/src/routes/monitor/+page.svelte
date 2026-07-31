<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
	import { Canvas } from '@threlte/core';
	import type { CameraControlsRef } from '@threlte/extras';
	import ControlPanel from '$lib/ControlPanel.svelte';
	import AntennaStatePanel from '$lib/AntennaStatePanel.svelte';
	import { cursorAzEl } from '$lib/stores/ui.svelte';

	let { data }: { data: { token: string } } = $props();

	let controls = $state.raw<CameraControlsRef>();

	let connectionState: 'connecting' | 'open' | 'closed' = $state('connecting');
	let socket: WebSocket | null = null;
	let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
	let destroyed = false;

	let time = $derived(daemonStatus.time ?? Date.now() / 1000);
	let telescopeAzEl = $derived(
		daemonStatus.rotor
			? [daemonStatus.rotor.az ?? 0, daemonStatus.rotor.el ?? 0]
			: [0, 0]
	);

	function connect(token: string) {
		connectionState = 'connecting';
		const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
		socket = new WebSocket(
			`${protocol}://${window.location.host}/ws/status?token=${encodeURIComponent(token)}`
		);

		socket.onopen = () => (connectionState = 'open');

		socket.onmessage = (event) => {
			const status = JSON.parse(event.data);
			Object.assign(daemonStatus, status);
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
	<p>Cursor Az/El: {cursorAzEl[0].toFixed(2)}°, {cursorAzEl[1].toFixed(2)}°</p>
	<p>Rotor Az/El: {telescopeAzEl[0].toFixed(2)}°, {telescopeAzEl[1].toFixed(2)}°</p>
	<div class="grid grid-cols-1 md:grid-cols-2 gap-4 mt-4">
		<AntennaStatePanel />
		<div class="border-2 border-gray-700 rounded-lg h-[80vh]">
			<Canvas>
				<ControlPanel bind:controls />
			</Canvas>
		</div>
	</div>
</div>
