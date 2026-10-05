"""Expose this robot's meshes and resolve its URDF for rosbridge clients only."""
import re
import runpy
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from std_msgs.msg import String
from rclpy.serialization import deserialize_message, serialize_message
from rosbridge_library.internal.subscribers import MultiSubscriber
import tornado.web


class RobotAssetHandler(tornado.web.StaticFileHandler):
    def set_default_headers(self):
        self.set_header('Access-Control-Allow-Origin', '*')
        self.set_header('Access-Control-Allow-Methods', 'GET, HEAD, OPTIONS')
        self.set_header('Access-Control-Allow-Private-Network', 'true')

    def options(self, path):
        self.set_status(204)
        self.finish()

    def validate_absolute_path(self, root, absolute_path):
        resolved = Path(absolute_path).resolve()
        # Symlink installs are supported, but only the model's mesh directory is served.
        if resolved not in allowed_mesh_files:
            raise tornado.web.HTTPError(403)
        return super().validate_absolute_path(root, absolute_path)


original_application = tornado.web.Application
mesh_root = str(Path(get_package_share_directory('mybot_description')) / 'meshes')
allowed_mesh_files = {path.resolve() for path in Path(mesh_root).rglob('*')
                      if path.is_file() and path.suffix.lower() in
                      {'.stl', '.dae', '.obj', '.mtl', '.png', '.jpg', '.jpeg'}}


class AssetApplication(original_application):
    def __init__(self, handlers=None, **kwargs):
        handlers = list(handlers or [])
        handlers.insert(0, (r'/assets/mybot_description/meshes/(.*)',
                            RobotAssetHandler, {'path': mesh_root}))
        super().__init__(handlers, **kwargs)


tornado.web.Application = AssetApplication
original_callback = MultiSubscriber.callback


def robot_callback(self, msg, callbacks=None):
    raw = isinstance(msg, bytes) and self.msg_class is String
    if raw:
        msg = deserialize_message(msg, String)
    if isinstance(msg, String) and '<robot' in msg.data:
        data = re.sub(r'package://mybot_description/meshes/',
                      'http://127.0.0.1:9090/assets/mybot_description/meshes/', msg.data)
        msg = String(data=data)
    if raw:
        msg = serialize_message(msg)
    return original_callback(self, msg, callbacks)


MultiSubscriber.callback = robot_callback
runpy.run_path(str(Path(get_package_prefix('rosbridge_server')) /
                   'lib/rosbridge_server/rosbridge_websocket'), run_name='__main__')
