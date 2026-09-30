"""Verify large CBOR-raw map/image messages through the live websocket."""
import asyncio
import json
from pathlib import Path
import cbor2
from tornado.websocket import websocket_connect


async def main():
    ws = await websocket_connect('ws://127.0.0.1:9090', connect_timeout=10,
                                 max_message_size=16000000)
    topics = {'/map': 'nav_msgs/msg/OccupancyGrid',
              '/global_costmap/costmap': 'nav_msgs/msg/OccupancyGrid',
              '/camera/image_raw': 'sensor_msgs/msg/Image'}
    received = {key: [] for key in topics}
    for topic, msg_type in topics.items():
        await ws.write_message(json.dumps({'op':'subscribe', 'topic':topic,
            'type':msg_type, 'compression':'cbor-raw',
            'throttle_rate':200, 'queue_length':1}))
    start = asyncio.get_running_loop().time()
    deadline = start+15
    while asyncio.get_running_loop().time() < deadline:
        try:
            raw = await asyncio.wait_for(ws.read_message(), timeout=2)
        except asyncio.TimeoutError:
            continue
        assert raw is not None, 'Connection closed'
        msg = cbor2.loads(raw) if isinstance(raw, bytes) else json.loads(raw)
        topic = msg.get('topic')
        if topic in received:
            assert isinstance(raw, bytes), 'Expected binary CBOR frame'
            received[topic].append((asyncio.get_running_loop().time()-start, len(raw)))
    ws.close()
    report = {key:{'messages':len(rows), 'max_bytes':max((r[1] for r in rows), default=0),
                  'observed_hz':(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else None}
              for key, rows in received.items()}
    assert all(value['messages'] for value in report.values()), report
    folder = Path(__file__).resolve().parents[1]/'log/foxglove-command-20260930'
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'visuals-check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    asyncio.run(main())
