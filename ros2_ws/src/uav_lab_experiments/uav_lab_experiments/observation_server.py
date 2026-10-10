"""Loopback WebSocket observation; slow browser clients cannot block ROS."""
import asyncio
import json
import threading
from websockets.legacy.server import serve
from websockets.exceptions import ConnectionClosed


class ObservationServer:
    def __init__(self,store,port=8765):
        self.store,self.port,self.clients=store,port,0
        self.ready=threading.Event();self.error=None;self.loop=None;self.stop=None
        self.thread=threading.Thread(target=self.run,name='web-observation',daemon=True)

    async def client(self,connection):
        if self.clients>=8:
            await connection.close(code=1013,reason='observation client limit');return
        self.clients+=1;seen={}
        try:
            while not connection.closed:
                for layer,(version,packet) in self.store.snapshot().items():
                    if seen.get(layer)==version:continue
                    # Every client has its own deadline and transport. No await
                    # occurs on the ROS thread or on another client's sender.
                    await asyncio.wait_for(connection.send(packet),timeout=.15)
                    seen[layer]=version
                await asyncio.sleep(.2)
        except asyncio.TimeoutError:
            await connection.close(code=1013,reason='browser observation stalled')
        except ConnectionClosed:
            pass
        finally:
            self.clients-=1

    async def main(self):
        self.loop=asyncio.get_running_loop();self.stop=asyncio.Event()
        async with serve(self.client,'127.0.0.1',self.port,
                         origins=['http://127.0.0.1:8080','http://localhost:8080',
                                  'http://127.0.0.1:8780','http://localhost:8780',
                                  'http://127.0.0.1:5173','http://localhost:5173'],
                         max_size=4096,max_queue=1,write_limit=65536,ping_interval=10,ping_timeout=10) as server:
            self.port=server.sockets[0].getsockname()[1]
            self.ready.set();await self.stop.wait()

    def run(self):
        try:asyncio.run(self.main())
        except Exception as error:
            self.error=error;self.ready.set()

    def start(self):
        self.thread.start()
        if not self.ready.wait(3):raise RuntimeError('observation socket startup timeout')
        if self.error:raise RuntimeError('observation socket unavailable: '+str(self.error))

    def close(self):
        if self.loop and self.stop:self.loop.call_soon_threadsafe(self.stop.set)
        self.thread.join(timeout=3)
