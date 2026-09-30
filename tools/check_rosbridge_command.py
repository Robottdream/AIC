"""Check rosbridge command delivery and missing-node queries without task motion."""
import asyncio
import json
from pathlib import Path
from tornado.websocket import websocket_connect


async def main():
    ws = await websocket_connect('ws://127.0.0.1:9090', connect_timeout=10)
    send = lambda value: ws.write_message(json.dumps(value))
    await send({'op':'subscribe', 'topic':'/command', 'type':'std_msgs/msg/String'})
    await send({'op':'call_service', 'service':'/rosapi/node_details',
                'args':{'node':'/nonexistent_probe_node'}, 'id':'missing-node'})
    await send({'op':'advertise', 'topic':'/command', 'type':'std_msgs/msg/String'})
    await asyncio.sleep(1)
    # Parser deliberately rejects this marker; it cannot trigger a robot task.
    await send({'op':'publish', 'topic':'/command', 'msg':{'data':'__foxglove_probe__'}})
    report = {'missing_node_query':False, 'command_loopback':False}
    deadline = asyncio.get_running_loop().time()+12
    while not all(report.values()) and asyncio.get_running_loop().time()<deadline:
        raw = await asyncio.wait_for(ws.read_message(), timeout=12)
        assert raw is not None, 'Websocket closed'
        result = json.loads(raw)
        if result.get('id') == 'missing-node':
            report['missing_node_query'] = result.get('result') is True and result.get('values') == {
                'subscribing':[], 'publishing':[], 'services':[]}
        if result.get('topic') == '/command' and result.get('msg',{}).get('data') == '__foxglove_probe__':
            report['command_loopback'] = True
    ws.close()
    assert all(report.values()), report
    folder = Path(__file__).resolve().parents[1]/'log/foxglove-command-20260930'
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'bridge-check.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    asyncio.run(main())
