"""Verify the exact robot sent to display clients over the live bridge."""
import asyncio
import json
import struct
import xml.etree.ElementTree as ET
from urllib.request import urlopen
import cbor2
from tornado.websocket import websocket_connect
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

async def main():
    ws=await websocket_connect('ws://127.0.0.1:9090',connect_timeout=10)
    await ws.write_message(json.dumps({'op':'subscribe','topic':'/robot_description','type':'std_msgs/msg/String','compression':'cbor-raw'}))
    try:
        while True:
            raw=await asyncio.wait_for(ws.read_message(),20)
            item=cbor2.loads(raw) if isinstance(raw,bytes) else json.loads(raw)
            if item.get('topic')=='/robot_description':
                text=deserialize_message(item['msg']['bytes'],String).data
                robot=ET.fromstring(text); break
    finally: ws.close()
    links={link.get('name') for link in robot.findall('link')}
    joints={joint.get('name') for joint in robot.findall('joint')}
    assert {'arm_yaw_joint','front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint'}<=joints
    assert {'laser_left_link','laser_right_link'}<=links
    assert 'laser_high_link' not in links
    base=robot.find("link[@name='base_link']")
    cylinder=base.find('collision/geometry/cylinder')
    assert cylinder is not None and abs(float(cylinder.get('radius'))-.24)<1e-6
    meshes=robot.findall('.//visual/geometry/mesh'); total=0; chassis=None
    for mesh in meshes:
        uri=mesh.get('filename'); assert uri.startswith('http://127.0.0.1:9090/assets/')
        with urlopen(uri,timeout=5) as response:
            assert response.headers.get('Access-Control-Allow-Origin')=='*'
            data=response.read()
        count=struct.unpack_from('<I',data,80)[0]; assert len(data)==84+50*count
        total+=count
        if uri.endswith('/base_link_blue_gray_chassis.stl'):
            vertices=[struct.unpack_from('<3f',data,84+i*50+12+j*12) for i in range(count) for j in range(3)]
            radius=max((x*x+y*y)**.5 for x,y,z in vertices)
            assert .239<radius<.242, f'Display mesh is not the circular chassis: {radius}'
            chassis={'uri':uri,'mesh_radius_m':radius,'triangles':count}
    assert chassis is not None
    print(json.dumps({'display_model':'round chassis + 4 omni wheels + arm yaw + side lidars','links':len(links),'visual_meshes':len(meshes),'triangles':total,'chassis':chassis,'all_assets_readable':True},ensure_ascii=False,indent=2))

if __name__=='__main__': asyncio.run(main())
