"""
Thin controller for calibration-related service interactions.
"""


class CalibrationController:
    """Wrap calibration/TCP service calls behind a small controller API."""

    def __init__(self, panel):
        self._panel = panel

    def log(self, text: str):
        if hasattr(self._panel, '_cal_log') and self._panel._cal_log is not None:
            self._panel._cal_log.append(text)

    def set_stickout(self, value_m: float, done_cb):
        if not self._panel._set_stickout_client.service_is_ready():
            self.log('ERROR: /tcp/set_stickout service not available')
            return False
        req = self._panel._set_stickout_client.srv_type.Request()
        req.data = value_m
        future = self._panel._set_stickout_client.call_async(req)
        future.add_done_callback(done_cb)
        return True

    def set_tcp_mode(self, mode: str, done_cb):
        if not self._panel._set_tcp_mode_client.service_is_ready():
            self.log('ERROR: /tcp/set_mode service not available')
            return False
        req = self._panel._set_tcp_mode_client.srv_type.Request()
        req.mode = mode
        future = self._panel._set_tcp_mode_client.call_async(req)
        future.add_done_callback(done_cb)
        return True

    def calibrate_stickout(self, req, done_cb):
        if not self._panel._calibrate_stickout_client.service_is_ready():
            self.log('ERROR: /calibration/calibrate_stickout not available')
            return False
        future = self._panel._calibrate_stickout_client.call_async(req)
        future.add_done_callback(done_cb)
        return True

    def calibrate_plate_plane(self, req, done_cb):
        if not self._panel._calibrate_plate_plane_client.service_is_ready():
            self.log('ERROR: /calibration/calibrate_plate_plane not available')
            return False
        future = self._panel._calibrate_plate_plane_client.call_async(req)
        future.add_done_callback(done_cb)
        return True