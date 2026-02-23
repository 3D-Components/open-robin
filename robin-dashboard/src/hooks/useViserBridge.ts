import { useEffect, useRef } from 'react';
import type { RobotCell } from '../types';

const WS_URL = 'ws://localhost:8082';
const THROTTLE_MS = 200;

/**
 * Sends robot state to the viser server via WebSocket.
 * The viser server maps segment progress to joint angles and
 * drives the 3D robot animation.
 */
export function useViserBridge(robot: RobotCell) {
    const wsRef = useRef<WebSocket | null>(null);
    const lastSentRef = useRef(0);
    const reconnectTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

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

    useEffect(() => {
        const now = Date.now();
        if (now - lastSentRef.current < THROTTLE_MS) return;

        const ws = wsRef.current;
        if (!ws || ws.readyState !== WebSocket.OPEN) return;

        const payload = {
            robot: {
                state: robot.state,
                segmentIndex: robot.segmentIndex,
                progressPct: robot.taskProgressPct,
            },
        };

        ws.send(JSON.stringify(payload));
        lastSentRef.current = now;
    }, [robot.state, robot.segmentIndex, robot.taskProgressPct]);
}
