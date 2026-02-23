import asyncio
from typing import AsyncIterator

import pytest
from httpx import AsyncClient, ASGITransport

from robin.alert_engine import app


@pytest.fixture(scope='session')
def anyio_backend():
    return 'asyncio'


@pytest.fixture(scope='session')
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture()
async def client() -> AsyncIterator[AsyncClient]:
    # httpx>=0.28 requires ASGITransport; lifespan arg not supported here
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url='http://test') as ac:
        yield ac


class DummyResponse:
    def __init__(self, status_code: int = 200, json_data=None, text: str = ''):
        self.status_code = status_code
        self._json = json_data
        self.text = text or ''

    def json(self):
        return self._json


__all__ = ['DummyResponse', 'client']
