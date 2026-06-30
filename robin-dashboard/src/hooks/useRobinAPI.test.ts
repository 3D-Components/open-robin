import { renderHook, waitFor } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import {
    checkDeviation,
    createProcess,
    fetchProcessAlerts,
    fetchProcessMeasurements,
    fetchTargetGeometry,
    getAIRecommendation,
    predictGeometry,
    publishRosIntent,
    reportProcessError,
    resumeProcess,
    selectModel,
    setInputParams,
    setProcessMode,
    setTarget,
    stopProcess,
    useAIModels,
    useHealth,
    useMeasurements,
    useProcessList,
    useProcessSnapshot,
    useProcessStatus,
    useTargetGeometry,
} from './useRobinAPI';

function mockFetchOnce(body: unknown, ok = true, status = 200) {
    const fetchMock = vi.fn().mockResolvedValue({
        ok,
        status,
        json: vi.fn().mockResolvedValue(body),
    });
    vi.stubGlobal('fetch', fetchMock);
    return fetchMock;
}

describe('ROBIN API helpers', () => {
    afterEach(() => {
        vi.unstubAllGlobals();
    });

    it('fetches measurements and alerts with encoded process ids and last filters', async () => {
        const fetchMock = mockFetchOnce({ status: 'ok' });

        await fetchProcessMeasurements('process 1', 25);
        await fetchProcessAlerts('process 1', 5);

        expect(fetchMock).toHaveBeenNthCalledWith(
            1,
            expect.stringContaining('/process/process%201/measurements?last=25'),
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            2,
            expect.stringContaining('/process/process%201/alerts?last=5'),
        );
    });

    it('raises a useful error for failed API responses', async () => {
        mockFetchOnce({ error: 'failed' }, false, 503);

        await expect(fetchProcessMeasurements('p1')).rejects.toThrow('HTTP 503');
    });

    it('posts process and model mutations with JSON bodies', async () => {
        const fetchMock = mockFetchOnce({ status: 'ok' });

        await selectModel('/models/demo.pt');
        await predictGeometry({ input_params: { current: 120 } });
        await setTarget('p1', 3.5, 4.2);
        await setProcessMode('p1', 'geometry_driven');
        await setInputParams('p1', { current: 125, voltage: 18 });
        await checkDeviation({ process_id: 'p1', tolerance: 0.1 });
        await getAIRecommendation({ process_id: 'p1', mode: 'parameter_driven' });
        await createProcess('p1', 'parameter_driven');

        expect(fetchMock).toHaveBeenNthCalledWith(
            1,
            expect.stringContaining('/ai/models/select'),
            expect.objectContaining({
                method: 'POST',
                body: JSON.stringify({ path: '/models/demo.pt' }),
            }),
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            3,
            expect.stringContaining('/process/p1/target'),
            expect.objectContaining({
                method: 'POST',
                body: JSON.stringify({ height: 3.5, width: 4.2 }),
            }),
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            8,
            expect.stringContaining('/create-process'),
            expect.objectContaining({
                method: 'POST',
                body: JSON.stringify({ process_id: 'p1', mode: 'parameter_driven' }),
            }),
        );
    });

    it('posts lifecycle and intent requests to the expected endpoints', async () => {
        const fetchMock = mockFetchOnce({ status: 'ok' });

        await stopProcess('process 1', 'operator pause');
        await resumeProcess('process 1');
        await fetchTargetGeometry('process 1');
        await publishRosIntent('START_PROCESS', { source: 'test' }, 'process 1');

        expect(fetchMock).toHaveBeenNthCalledWith(
            1,
            expect.stringContaining('/process/process%201/stop?reason=operator%20pause'),
            { method: 'POST' },
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            2,
            expect.stringContaining('/process/process%201/resume'),
            { method: 'POST' },
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            3,
            expect.stringContaining('/process/process%201/target'),
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            4,
            expect.stringContaining('/intent'),
            expect.objectContaining({
                method: 'POST',
                body: JSON.stringify({
                    intent: 'START_PROCESS',
                    process_id: 'process 1',
                    data: { source: 'test' },
                }),
            }),
        );
    });

    it('reports a process error with the abort message and surfaces failures', async () => {
        const fetchMock = mockFetchOnce({ status: 'recorded' });

        // Explicit message + reason
        await reportProcessError('process 1', 'Aborted', 'operator_abort');
        // Defaults applied when only the process id is given
        await reportProcessError('p2');

        expect(fetchMock).toHaveBeenNthCalledWith(
            1,
            expect.stringContaining('/process/process%201/error'),
            expect.objectContaining({
                method: 'POST',
                body: JSON.stringify({ message: 'Aborted', reason: 'operator_abort' }),
            }),
        );
        expect(fetchMock).toHaveBeenNthCalledWith(
            2,
            expect.stringContaining('/process/p2/error'),
            expect.objectContaining({
                method: 'POST',
                body: JSON.stringify({
                    message: 'Process aborted by operator; robot returned to home.',
                    reason: 'operator_abort',
                }),
            }),
        );
    });

    it('raises a useful error when reporting a process error fails', async () => {
        mockFetchOnce({ error: 'down' }, false, 500);

        await expect(reportProcessError('p1')).rejects.toThrow('HTTP 500');
    });

    it('polls health and process list endpoints into hook state', async () => {
        const fetchMock = vi
            .fn()
            .mockResolvedValueOnce({
                ok: true,
                status: 200,
                json: vi.fn().mockResolvedValue({ status: 'healthy' }),
            })
            .mockResolvedValueOnce({
                ok: true,
                status: 200,
                json: vi.fn().mockResolvedValue({ status: 'ok', processes: [], total_count: 0 }),
            });
        vi.stubGlobal('fetch', fetchMock);

        const health = renderHook(() => useHealth(5000));
        await waitFor(() => expect(health.result.current.loading).toBe(false));
        expect(health.result.current.data).toEqual({ status: 'healthy' });
        health.unmount();

        const processList = renderHook(() => useProcessList(5000));
        await waitFor(() => expect(processList.result.current.loading).toBe(false));
        expect(processList.result.current.data).toEqual({ status: 'ok', processes: [], total_count: 0 });
        processList.unmount();

        expect(fetchMock).toHaveBeenNthCalledWith(1, expect.stringContaining('/health'));
        expect(fetchMock).toHaveBeenNthCalledWith(2, expect.stringContaining('/processes/list'));
    });

    it('polls process-specific endpoints when a process id is available', async () => {
        const fetchMock = vi
            .fn()
            .mockResolvedValue({
                ok: true,
                status: 200,
                json: vi.fn().mockResolvedValue({ status: 'ok' }),
            });
        vi.stubGlobal('fetch', fetchMock);

        const measurements = renderHook(() => useMeasurements('process 1', 10, 5000));
        await waitFor(() => expect(measurements.result.current.loading).toBe(false));
        measurements.unmount();

        const status = renderHook(() => useProcessStatus('process 1', 5000));
        await waitFor(() => expect(status.result.current.loading).toBe(false));
        status.unmount();

        const snapshot = renderHook(() => useProcessSnapshot('process 1', 5000));
        await waitFor(() => expect(snapshot.result.current.loading).toBe(false));
        snapshot.unmount();

        const target = renderHook(() => useTargetGeometry('process 1', 5000));
        await waitFor(() => expect(target.result.current.loading).toBe(false));
        target.unmount();

        expect(fetchMock).toHaveBeenNthCalledWith(
            1,
            expect.stringContaining('/process/process%201/measurements?last=10'),
        );
        expect(fetchMock).toHaveBeenNthCalledWith(2, expect.stringContaining('/process/process%201/status'));
        expect(fetchMock).toHaveBeenNthCalledWith(3, expect.stringMatching(/\/process\/process%201$/));
        expect(fetchMock).toHaveBeenNthCalledWith(4, expect.stringContaining('/process/process%201/target'));
    });

    it('polls AI models and exposes fetch errors in hook state', async () => {
        const fetchMock = vi
            .fn()
            .mockResolvedValueOnce({
                ok: true,
                status: 200,
                json: vi.fn().mockResolvedValue({ models: [], active_model: null }),
            })
            .mockResolvedValueOnce({
                ok: false,
                status: 500,
                json: vi.fn(),
            });
        vi.stubGlobal('fetch', fetchMock);

        const models = renderHook(() => useAIModels());
        await waitFor(() => expect(models.result.current.loading).toBe(false));
        expect(models.result.current.data).toEqual({ models: [], active_model: null });
        models.unmount();

        const health = renderHook(() => useHealth(5000));
        await waitFor(() => expect(health.result.current.loading).toBe(false));
        expect(health.result.current.error).toBe('HTTP 500');
        health.unmount();
    });
});
