import statistics
import time

import pytest


def _time_many(fn, iterations: int = 20):
    durs = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        fn()
        durs.append((time.perf_counter() - t0) * 1000.0)  # ms
    return durs


@pytest.mark.asyncio
async def test_perf_root_endpoint(client):
    async def once():
        resp = await client.get('/')
        assert resp.status_code == 200

    # Warmup
    await once()

    durs = []
    for _ in range(20):
        t0 = time.perf_counter()
        await once()
        durs.append((time.perf_counter() - t0) * 1000.0)

    mean_ms = statistics.mean(durs)
    p95_ms = statistics.quantiles(durs, n=20)[-1]

    # Generous thresholds suitable for CI environments
    assert mean_ms < 500.0
    assert p95_ms < 1000.0


@pytest.mark.asyncio
async def test_perf_ai_predict_smoke(client):
    payload = {'wireSpeed': 2.5, 'current': 110.0, 'voltage': 17.0}

    async def once():
        resp = await client.post('/ai/models/predict', json=payload)
        assert resp.status_code == 200

    # Warmup
    await once()

    durs = []
    for _ in range(10):
        t0 = time.perf_counter()
        await once()
        durs.append((time.perf_counter() - t0) * 1000.0)

    mean_ms = statistics.mean(durs)
    assert (
        mean_ms < 800.0
    )  # prediction includes minimal model work or heuristic
