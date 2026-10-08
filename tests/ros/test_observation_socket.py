import asyncio
import pytest
from websockets.legacy.client import connect
from websockets.exceptions import InvalidStatusCode
from uav_lab_experiments.observatory import LatestFrames
from uav_lab_experiments.observation_server import ObservationServer


def test_actual_socket_disconnect_and_reconnect_do_not_queue_old_frames():
    store=LatestFrames();server=ObservationServer(store,port=0);server.start()
    async def exercise():
        uri=f'ws://127.0.0.1:{server.port}'
        store.put('raw',b'first')
        async with connect(uri,origin='http://127.0.0.1:8080') as socket:
            assert await asyncio.wait_for(socket.recv(),1)==b'first'
        for i in range(100):store.put('raw',str(i).encode())
        async with connect(uri,origin='http://127.0.0.1:8080') as socket:
            assert await asyncio.wait_for(socket.recv(),1)==b'99'
            store.put('raw',b'new')
            assert await asyncio.wait_for(socket.recv(),1)==b'new'
        with pytest.raises(InvalidStatusCode):
            async with connect(uri,origin='https://unrelated.example'):pass
    try:asyncio.run(exercise())
    finally:server.close()
    assert not server.thread.is_alive()
