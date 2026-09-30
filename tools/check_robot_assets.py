"""Verify live URDF mesh retrieval through the existing rosbridge connection."""
import asyncio
import json
import struct
import cbor2
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
import xml.etree.ElementTree as ET
from urllib.request import urlopen
from tornado.websocket import websocket_connect


async def main():
    ws = await websocket_connect('ws://127.0.0.1:9090', connect_timeout=10)
    await ws.write_message(json.dumps({'op': 'subscribe', 'topic': '/robot_description',
                                      'type': 'std_msgs/msg/String', 'compression': 'cbor-raw'}))
    try:
        while True:
            raw = await asyncio.wait_for(ws.read_message(), 20)
            item = cbor2.loads(raw) if isinstance(raw, bytes) else json.loads(raw)
            if item.get('topic') == '/robot_description':
                robot = ET.fromstring(deserialize_message(item['msg']['bytes'], String).data)
                break
    finally:
        ws.close()
    meshes = robot.findall('.//visual/geometry/mesh')
    assert meshes, 'Robot has no visual meshes'
    triangles = 0
    for mesh in meshes:
        uri = mesh.attrib['filename']
        assert uri.startswith('http://127.0.0.1:9090/assets/'), uri
        with urlopen(uri, timeout=5) as response:
            assert response.headers.get('Access-Control-Allow-Origin') == '*'
            data = response.read()
        count = struct.unpack_from('<I', data, 80)[0]
        assert len(data) == 84 + 50 * count, uri
        triangles += count
    print(json.dumps({'visual_meshes': len(meshes), 'triangles': triangles,
                      'links': len(robot.findall('link')), 'all_assets_ok': True}))


if __name__ == '__main__':
    asyncio.run(main())
