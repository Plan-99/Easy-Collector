# -*- coding: utf-8 -*-
"""Socket.IO control client for dual-arm sim tests.

Mirrors how the operator UI drives a robot (frontend/src/composables/useRobot.js
+ TeleopConsole.vue keyboard teleop): it subscribes to `robot_status_<id>` for
live joint_states / ee_pos, and emits `move_robot_ee_delta` / `move_robot_joint_delta`.

This is exactly the path keyboard teleoperation uses, so a passing test means
keyboard control of the (dual-)arm works end-to-end.
"""
import threading
import time

import socketio


class RobotLink:
    """Live link to one robot over Socket.IO: status in, control out."""

    def __init__(self, backend_url, robot_id):
        self.backend_url = backend_url.rstrip('/')
        self.robot_id = robot_id
        self._sio = socketio.Client(reconnection=True, logger=False, engineio_logger=False)
        self._lock = threading.Lock()
        self.joint_states = None      # list[float]
        self.ee_pos = None            # dict[ee_name -> [x,y,z,ax,ay,az]]
        self.connected = False

        @self._sio.on(f'robot_status_{robot_id}')
        def _on_status(data):          # noqa: ANN001
            with self._lock:
                if data.get('joint_states') is not None:
                    self.joint_states = list(data['joint_states'])
                if data.get('ee_pos') is not None:
                    self.ee_pos = dict(data['ee_pos'])
                self.connected = bool(data.get('connected'))

    def connect(self):
        self._sio.connect(self.backend_url, wait=True, transports=['websocket', 'polling'])
        return self

    def close(self):
        try:
            self._sio.disconnect()
        except Exception:  # noqa: BLE001
            pass

    # --- snapshots --------------------------------------------------------
    def snapshot(self):
        with self._lock:
            js = list(self.joint_states) if self.joint_states is not None else None
            ee = {k: list(v) for k, v in self.ee_pos.items()} if self.ee_pos else None
        return js, ee

    def wait_for_state(self, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            js, ee = self.snapshot()
            if js is not None:
                return js, ee
            time.sleep(0.1)
        raise RuntimeError(f'robot {self.robot_id}: no joint_states within {timeout}s')

    # --- control out ------------------------------------------------------
    def _robot_ref(self):
        return {'id': self.robot_id}

    def send_ee_delta(self, ee_name, delta6):
        """delta6 = [dx,dy,dz,drx,dry,drz] for one end-effector."""
        self._sio.emit('move_robot_ee_delta',
                       {'robot': self._robot_ref(), 'delta_pos': {ee_name: list(delta6)}})

    def send_joint_delta(self, delta_vec):
        self._sio.emit('move_robot_joint_delta',
                       {'robot': self._robot_ref(), 'delta_pos': list(delta_vec)})


def pulse_ee_delta(link: RobotLink, ee_name, axis, step, n=12, dt=0.08):
    """Repeatedly nudge one EE axis (mimics holding a teleop key).

    axis: index 0..5 into [x,y,z,ax,ay,az]. Returns (ee_before, ee_after) for
    that end-effector.
    """
    _, ee0 = link.wait_for_state()
    before = list(ee0[ee_name]) if ee0 and ee_name in ee0 else None
    delta = [0.0] * 6
    delta[axis] = step
    for _ in range(n):
        link.send_ee_delta(ee_name, delta)
        time.sleep(dt)
    time.sleep(0.5)
    _, ee1 = link.snapshot()
    after = list(ee1[ee_name]) if ee1 and ee_name in ee1 else None
    return before, after


def max_abs_pos_change(before, after):
    """Max |Δ| over the xyz translation components of an EE pose."""
    if before is None or after is None:
        return 0.0
    return max(abs(after[i] - before[i]) for i in range(3))


def joint_subset_change(js_before, js_after, indices):
    """Max |Δ| over a subset of joint indices."""
    if js_before is None or js_after is None:
        return 0.0
    return max(abs(js_after[i] - js_before[i]) for i in indices)
