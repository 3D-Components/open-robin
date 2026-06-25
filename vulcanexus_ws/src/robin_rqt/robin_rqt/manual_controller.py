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

    def _call(self, srv_type, service_name: str, value, done_cb) -> bool:
        client = self._get_client(srv_type, service_name)
        if not client.service_is_ready():
            return False
        req = srv_type.Request()
        req.data = value
        future = client.call_async(req)
        future.add_done_callback(done_cb)
        return True

    def set_working_mode(self, mode_val: int, done_cb):
        return self._call(SetInt32, '/wago/in/working_mode', int(mode_val), done_cb)

    def set_float(self, service_name: str, value: float, done_cb):
        return self._call(SetFloat32Srv, service_name, float(value), done_cb)

    def set_robot_ready(self, ready: bool, done_cb):
        return self._call(SetBool, '/wago/in/robot_ready', bool(ready), done_cb)