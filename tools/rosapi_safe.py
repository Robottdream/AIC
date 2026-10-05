"""Run installed rosapi with a guard for disappearing graph nodes."""
import runpy
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from rosapi import proxy

original = proxy.get_node_info


def get_node_info(name, include_hidden=False):
    result = original(name, include_hidden=include_hidden)
    return result if result is not None else ([], [], [])


proxy.get_node_info = get_node_info
runpy.run_path(str(Path(get_package_prefix('rosapi'))/'lib/rosapi/rosapi_node'),
               run_name='__main__')
