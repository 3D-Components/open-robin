import { useEffect, useRef } from 'react';
import type { RobotCell } from '../types';

const WS_URL = 'ws://localhost:8082';
const THROTTLE_MS = 200;

/**
 * Sends robot state to the viser server via WebSocket.
 * The viser server maps segment progress to joint angles and
 * drives the 3D robot animation.
 */
export function useViserBridge(
    robots: Record<RobotCell['id'], RobotCell>,
) {
    const wsRef = useRef<WebSocket | null>(null);
    const lastSentRef = useRef(0);
    const reconnectTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

    // Connect / reconnect
    useEffect(() => {
        function connect() {
            const ws = new WebSocket(WS_URL);
            wsRef.current = ws;

            ws.onopen = () => {
                console.log('[ViserBridge] connected');
            };

            ws.onclose = () => {
                console.log('[ViserBridge] disconnected, reconnecting in 2s…');
                wsRef.current = null;
                reconnectTimer.current = setTimeout(connect, 2000);
            };

            ws.onerror = () => {
                ws.close();
            };
        }

        connect();

        return () => {
            clearTimeout(reconnectTimer.current);
            wsRef.current?.close();
            wsRef.current = null;
        };
    }, []);

    // Send state updates (throttled)
    useEffect(() => {
        const now = Date.now();
        if (now - lastSentRef.current < THROTTLE_MS) return;

        const ws = wsRef.current;
        if (!ws || ws.readyState !== WebSocket.OPEN) return;

        const payload = {
            robotA: {
                state: robots.robotA.state,
                segmentIndex: robots.robotA.segmentIndex,
                progressPct: robots.robotA.taskProgressPct,
            },
            robotB: {
                state: robots.robotB.state,
                segmentIndex: robots.robotB.segmentIndex,
                progressPct: robots.robotB.taskProgressPct,
            },
        };

        ws.send(JSON.stringify(payload));
        lastSentRef.current = now;
    }, [
        robots.robotA.state,
        robots.robotA.segmentIndex,
        robots.robotA.taskProgressPct,
        robots.robotB.state,
        robots.robotB.segmentIndex,
        robots.robotB.taskProgressPct,
    ]);
}
