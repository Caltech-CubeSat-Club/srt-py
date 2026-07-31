<script lang="ts">
  import { daemonStatus } from '$lib/stores/daemonStatus.svelte';
  import type { DriverState, CalSts } from '$lib/generated/types';

  // Color mapping for FSM states
  const stateColorMap: Record<string, string> = {
    disconnected: 'bg-gray-500',
    connecting: 'bg-gray-500',
    startup_sync: 'bg-yellow-500',
    ready: 'bg-blue-500',
    slewing: 'bg-cyan-500',
    tracking: 'bg-green-500',
    calibrating: 'bg-yellow-500',
    fault: 'bg-red-600',
    recovering: 'bg-yellow-500',
    shutdown: 'bg-gray-800',
  };

  const calStsColorMap: Record<CalSts, string> = {
    'Not Calibrated': 'bg-red-600',
    'Calibrating Now': 'bg-yellow-500',
    'Calibration OK': 'bg-green-500',
  };

  function getBadgeColor(fsmState?: DriverState): string {
    if (!fsmState) return 'bg-gray-500';
    return stateColorMap[fsmState] || 'bg-gray-500';
  }

  function getCalColor(calSts: CalSts): string {
    return calStsColorMap[calSts] || 'bg-gray-500';
  }

  function getBoolColor(value: boolean | undefined, trueColor: string, falseColor: string): string {
    if (!value) return 'bg-gray-400';
    return value ? trueColor : falseColor;
  }

  let ampCurrents = $derived(daemonStatus.rotor?.amp_currents ?? {});

  let modeColor = $derived(daemonStatus.rotor?.loop_mode === 'Track' ? 'bg-green-500' : daemonStatus.rotor?.loop_mode === 'Stop' ? 'bg-gray-500' : 'bg-yellow-500');

  let limitSwitches = $derived({
    'El↑ Pre': daemonStatus.rotor?.el_up_pre,
    'El↓ Pre': daemonStatus.rotor?.el_dn_pre,
    'El↑ Fin': daemonStatus.rotor?.el_up_fin,
    'El↓ Fin': daemonStatus.rotor?.el_dn_fin,
    'Az CW Pre': daemonStatus.rotor?.az_cw_pre,
    'Az CCW Pre': daemonStatus.rotor?.az_ccw_pre,
    'Az CW Fin': daemonStatus.rotor?.az_cw_fin,
    'Az CCW Fin': daemonStatus.rotor?.az_ccw_fin,
  });

  let anyLimitActive = $derived(Object.values(limitSwitches).some(Boolean));
</script>

<div class="rounded-lg border border-gray-200 bg-white p-4 shadow-sm mb-2">
  <!-- Card Header -->
  <div class="mb-4">
    <h5 class="text-lg font-semibold text-gray-900">Antenna State</h5>
  </div>

  <!-- Card Body -->
  <div class="space-y-4">
    <!-- Top Badges Row -->
    <div class="flex flex-wrap gap-2 mb-2">
      <span
        class={`${getBadgeColor(daemonStatus.rotor?.fsm_state)} text-white text-xs font-medium px-3 py-1 rounded-full`}
      >
        {daemonStatus.rotor?.fsm_state?.toUpperCase() ?? 'Unknown'}
      </span>
      <span
        class={`${getCalColor(daemonStatus.rotor?.cal_sts ?? 'Not Calibrated')} text-white text-xs font-medium px-3 py-1 rounded-full`}
      >
        {daemonStatus.rotor?.cal_sts ?? 'Unknown'}
      </span>
      <span
        class={`${modeColor} text-white text-xs font-medium px-3 py-1 rounded-full`}
      >
        Loop: {daemonStatus.rotor?.loop_mode ?? 'Unknown'}
      </span>
      {#if daemonStatus.rotor?.safe_mode}
        <span
          class="bg-yellow-500 text-white text-xs font-medium px-3 py-1 rounded-full"
        >
          SAFE MODE
        </span>
      {/if}
      {#if daemonStatus.rotor?.sim_mode}
        <span
          class="bg-cyan-500 text-white text-xs font-medium px-3 py-1 rounded-full"
        >
          SIM
        </span>
      {/if}
    </div>

    <!-- Position Row -->
    <div class="grid grid-cols-4 gap-4 mb-2">
      <div>
        <p class="text-xs text-gray-600 uppercase tracking-wide mb-1">Az</p>
        <p class="font-mono text-lg font-semibold">{(daemonStatus.rotor?.az ?? 0).toFixed(3)}°</p>
        <p class="text-xs text-gray-600 font-mono">err: {(daemonStatus.rotor?.az_err ?? 0).toFixed(4)}m°</p>
      </div>
      <div>
        <p class="text-xs text-gray-600 uppercase tracking-wide mb-1">El</p>
        <p class="font-mono text-lg font-semibold">{(daemonStatus.rotor?.el ?? 0).toFixed(3)}°</p>
        <p class="text-xs text-gray-600 font-mono">err: {(daemonStatus.rotor?.el_err ?? 0).toFixed(4)}m°</p>
      </div>
      <div>
        <p class="text-xs text-gray-600 uppercase tracking-wide mb-1">Az Brake</p>
        {#if daemonStatus.rotor?.az_brake !== null}
          <div
            class={`${getBoolColor(daemonStatus.rotor?.az_brake, 'bg-red-600', 'bg-green-500')} text-white text-xs font-medium px-2 py-1 rounded inline-block`}
          >
            {daemonStatus.rotor?.az_brake ? 'ON (locked)' : 'OFF (free)'}
          </div>
        {:else}
          <p class="text-gray-400">?</p>
        {/if}
      </div>
      <div>
        <p class="text-xs text-gray-600 uppercase tracking-wide mb-1">El Brake</p>
        {#if daemonStatus.rotor?.el_brake !== null}
          <div
            class={`${getBoolColor(daemonStatus.rotor?.el_brake, 'bg-red-600', 'bg-green-500')} text-white text-xs font-medium px-2 py-1 rounded inline-block`}
          >
            {daemonStatus.rotor?.el_brake ? 'ON (locked)' : 'OFF (free)'}
          </div>
        {:else}
          <p class="text-gray-400">?</p>
        {/if}
      </div>
    </div>

    <!-- E-Stop and Retries Row -->
    <div class="space-y-2 mb-2">
      <div
        class={`${getBoolColor(daemonStatus.rotor?.estop, 'bg-red-600', 'bg-green-500')} text-white text-xs font-medium px-3 py-1 rounded inline-block`}
      >
        {daemonStatus.rotor?.estop ? '⚠ Physical E-STOP Button ACTIVE' : 'Physical E-Stop Button: clear'}
      </div>
      <div class="ml-2">
        <div
          class={`${daemonStatus.rotor?.retry_count ?? 0 > 0 ? 'bg-yellow-500' : 'bg-green-500'} text-white text-xs font-medium px-3 py-1 rounded inline-block`}
        >
          {daemonStatus.rotor?.retry_count ?? 0 > 0 ? `Retries: ${daemonStatus.rotor?.retry_count ?? 0}` : 'Comms OK'}
        </div>
      </div>
    </div>

    <!-- Limit Switches Section -->
    <div class="mb-2">
      <p
        class={`text-xs font-semibold mb-2 ${anyLimitActive ? 'text-red-500' : 'text-gray-700'}`}
      >
        Limit Switches {anyLimitActive ? '— ⚠ ACTIVE' : '— clear'}
      </p>
      <div class="flex flex-wrap gap-2">
        {#each Object.entries(limitSwitches) as [name, active]}
          <div
            class={`${
              active ? 'bg-red-600' : 'bg-gray-400'
            } text-white text-xs font-medium px-2 py-1 rounded`}
          >
            {active ? `⚠ ${name}` : name}
          </div>
        {/each}
      </div>
    </div>

    <!-- Amplifier Currents Section -->
    <div class="mb-2">
      <p class="text-xs font-semibold text-gray-700 mb-2 uppercase tracking-wide">Amplifier Currents</p>
      <div class="space-y-1">
        {#each ['2A01', '2A02', '2A03'] as ampId}
          {@const info = (ampCurrents ?? {})[ampId] || {}}
          {@const cmd = info.commanded ?? -999999}
          {@const act = info.actual ?? -999999}
          {@const cmdStr = cmd === -999999 ? '—' : String(cmd)}
          {@const actStr = act === -999999 ? '—' : String(act)}
          <div class="grid grid-cols-12 gap-2 text-xs">
            <div class="col-span-2 text-gray-600">{ampId}</div>
            <div class="col-span-5 font-mono text-gray-700">cmd: {cmdStr}</div>
            <div class="col-span-5 font-mono text-gray-700">act: {actStr}</div>
          </div>
        {/each}
      </div>
    </div>

    <!-- Error Section -->
    {#if daemonStatus.rotor?.last_error}
      <div class="bg-red-100 border-l-4 border-red-600 text-red-700 p-2 mb-2 text-xs">
        <strong>Last error: </strong>{daemonStatus.rotor?.last_error}
      </div>
    {/if}

    <!-- Metadata -->
    <p class="text-xs text-gray-500">Last transition: {daemonStatus.rotor?.last_transition}</p>
  </div>
</div>