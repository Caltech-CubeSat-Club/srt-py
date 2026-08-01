import type {
    DaemonStatus
} from '$lib/generated/types';

export const daemonStatus = $state<DaemonStatus>({});

export const connection = $state({
    state: 'closed' as 'connecting' | 'open' | 'closed'
});

export function updateConnState(state: 'connecting' | 'open' | 'closed') {
    connection.state = state;
}