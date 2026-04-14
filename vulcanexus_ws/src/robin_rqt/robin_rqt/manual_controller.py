"""
Thin controller for manual-tab service interactions.
"""

from std_srvs.srv import SetBool
from robin_interfaces.srv import SetFloat32 as SetFloat32Srv, SetInt32


class ManualController:
    def __init__(self, node):
        self._node = node
        self._clients = {}

    def _get_client(self, srv_type, service_name: str):
        key = (srv_type, service_name)
        client = self._clients.get(key)
        if client is None:
            client = self._node.create_client(srv_type, service_name)
            self._clients[key] = client
        return client

    def set_working_mode(self, mode_val: int, done_cb):
        client = self._get_client(SetInt32, '/wago/in/working_mode')
        if not client.service_is_ready():
            return False
        req = SetInt32.Request()
        req.data = int(mode_val)
        future = client.call_async(req)
        future.add_done_callback(done_cb)
        return True

    def set_float(self, service_name: str, value: float, done_cb):
        client = self._get_client(SetFloat32Srv, service_name)
        if not client.service_is_ready():
            return False
        req = SetFloat32Srv.Request()
        req.data = float(value)
        future = client.call_async(req)
        future.add_done_callback(done_cb)
        return True

    def set_robot_ready(self, ready: bool, done_cb):
        client = self._get_client(SetBool, '/wago/in/robot_ready')
        if not client.service_is_ready():
            return False
        req = SetBool.Request()
        req.data = bool(ready)
        future = client.call_async(req)
        future.add_done_callback(done_cb)
        return True