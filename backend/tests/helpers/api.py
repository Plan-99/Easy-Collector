# -*- coding: utf-8 -*-
"""Shared REST + Socket.IO helpers for the dual-arm sim tests.

These tests are CLIENTS: they talk to a running backend over host networking
(localhost:5000), exactly like the existing test_e2e_pipeline.py. They assume
the backend + ROS2 bridge are already up (see backend/tests/README.md).
"""
import time

import requests

try:
    import socketio
except ImportError:  # pragma: no cover
    raise SystemExit('python-socketio[client] required: pip install "python-socketio[client]"')


DEFAULT_BACKEND = 'http://127.0.0.1:5000'


class Backend:
    """Tiny REST wrapper around the EasyTrainer /api surface."""

    def __init__(self, base_url=DEFAULT_BACKEND):
        self.base_url = base_url.rstrip('/')

    def _url(self, path):
        return f'{self.base_url}/api{path}' if path.startswith('/') else f'{self.base_url}/api/{path}'

    def _call(self, method, path, **kwargs):
        url = self._url(path)
        timeout = kwargs.pop('timeout', 30)
        resp = requests.request(method, url, timeout=timeout, **kwargs)
        if resp.status_code >= 400:
            raise RuntimeError(f'{method} {url} -> HTTP {resp.status_code}: {resp.text[:300]}')
        try:
            return resp.json()
        except ValueError:
            return {'raw': resp.text}

    def get(self, path, **kw):
        return self._call('GET', path, **kw)

    def post(self, path, **kw):
        return self._call('POST', path, **kw)

    def put(self, path, **kw):
        return self._call('PUT', path, **kw)

    def delete(self, path, **kw):
        return self._call('DELETE', path, **kw)


def start_sim_test(api: Backend, env: str, show_viewer=False, max_wait_topics=90.0):
    """POST /api/<env>:start, then wait until the sim's topics are live.

    Returns the start response dict (robot_ids, sensor_ids, assembly_id,
    workspace_id). Raises on failure / timeout.
    """
    res = api.post(f'/{env}:start', json={'show_viewer': bool(show_viewer)})
    if res.get('status') != 'success':
        raise RuntimeError(f'{env}:start failed: {res}')
    for key in ('robot_ids', 'sensor_ids', 'assembly_id', 'workspace_id'):
        if res.get(key) in (None, [], {}):
            raise RuntimeError(f'{env}:start did not return {key}: {res}')

    deadline = time.time() + max_wait_topics
    last = None
    while time.time() < deadline:
        last = api.get(f'/{env}/status')
        if last.get('has_topics'):
            print(f'  [{env}] sim ready: robots={res["robot_ids"]} '
                  f'sensors={res["sensor_ids"]} assembly={res["assembly_id"]} '
                  f'workspace={res["workspace_id"]} (running={last.get("running")})')
            return res
        time.sleep(1.0)
    raise RuntimeError(f'{env} topics did not appear within {max_wait_topics}s: {last}')


def stop_sim_test(api: Backend, env: str):
    try:
        api.post(f'/{env}:stop')
    except Exception as e:  # noqa: BLE001
        print(f'  [WARN] {env}:stop failed: {e}')


def subscribe_robot(api: Backend, robot_id):
    """Register a RemoteAgent in current_app.agents + start topic subscription.

    Required before any move_robot_* socket event will resolve the agent.
    """
    res = api.post(f'/robot/{robot_id}/:subscribe_robot')
    if res.get('status') != 'success':
        raise RuntimeError(f'subscribe_robot({robot_id}) failed: {res}')


def get_robot(api: Backend, robot_id):
    res = api.get(f'/robot/{robot_id}')
    return res.get('robot', res)
