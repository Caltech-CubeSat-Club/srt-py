<script lang="ts">
	import { onMount, onDestroy } from 'svelte';
	import { daemonStatus, updateConnState } from '$lib/stores/daemonStatus.svelte';
	import { Canvas } from '@threlte/core';
	import type { CameraControlsRef } from '@threlte/extras';
	import ControlPanel from '$lib/ControlPanel.svelte';
	import AntennaStatePanel from '$lib/AntennaStatePanel.svelte';

	let { data }: { data: { token: string } } = $props();

	let controls = $state.raw<CameraControlsRef>();

	let socket: WebSocket | null = null;
	let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
	let destroyed = false;

	function connect(token: string) {
		updateConnState('connecting');
		const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
		socket = new WebSocket(
			`${protocol}://${window.location.host}/ws/status?token=${encodeURIComponent(token)}`
		);

		socket.onopen = () => updateConnState('open');

		socket.onmessage = (event) => {
			const status = JSON.parse(event.data);
			Object.assign(daemonStatus, status);
		};

		socket.onclose = () => {
			updateConnState('closed');
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
	<div class="grid grid-cols-1 md:grid-cols-3 gap-4 mt-4">
		<AntennaStatePanel />
		<div class="border-2 border-gray-700 h-[90vh] md:col-span-2">
			<Canvas>
				<ControlPanel bind:controls />
			</Canvas>
		</div>
	</div>
</div>
