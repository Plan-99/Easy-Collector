# -*- coding: utf-8 -*-
"""
RemoteObs — 센서 구독 gRPC proxy + shm 직접 읽기.

gRPC는 센서 구독 시작/종료(lifecycle)에만 사용.
이미지 데이터는 /dev/shm/easytrainer/에서 직접 읽음 (zero-copy).

여러 caller(workspace watch + record_episode 등)가 동일 sensor set 을 동시에
요구하는 경우, 중복 StartSensors RPC 를 보내지 않도록 process-local refcount
캐시를 둔다. 첫 caller 가 실제 ROS subscriber 를 띄우고, 이후 caller 는 같은
session_id 를 재사용한다. 마지막 caller 가 stop() 할 때 실제로 StopSensors 가
호출된다.
"""
import json
import threading
import time

from .generated import robot_bridge_pb2 as pb
from .client import get_bridge_client
from .image_reader import ImageBridgeReader


# process-local session cache: {sorted_sensor_ids_tuple: {'id': session_id, 'refs': int}}
_SESSION_CACHE: dict[tuple, dict] = {}
_CACHE_LOCK = threading.Lock()


def _cache_key(sensors):
    """sensor 식별자만으로 키 생성 — 동일 sensor set 이면 같은 키."""
    return tuple(sorted(int(s['id']) for s in sensors if 'id' in s))


class RemoteObs:
    def __init__(self, sensors):
        self.sensors = sensors
        self._image_reader = ImageBridgeReader()
        self._session_id = None
        self._cache_key = None

        if not sensors:
            return

        key = _cache_key(sensors)
        with _CACHE_LOCK:
            entry = _SESSION_CACHE.get(key)
            if entry is not None:
                entry['refs'] += 1
                self._session_id = entry['id']
                self._cache_key = key
                print(f"[RemoteObs] Reuse sensor session: id={self._session_id} refs={entry['refs']}", flush=True)
                return

            # 신규 — RPC 보내고 cache 등록 (lock 안에서 호출해야 동시 진입 시 중복 방지)
            client = get_bridge_client()
            result = client.obs.StartSensors(
                pb.SensorConfig(sensors_json=json.dumps(sensors))
            )
            self._session_id = result.id
            self._cache_key = key
            _SESSION_CACHE[key] = {'id': result.id, 'refs': 1}
            print(f"[RemoteObs] Sensor session started: id={self._session_id}", flush=True)

    def get_images(self):
        """Read latest sensor images from shared memory. No gRPC call."""
        image_dict = {}
        for sensor in self.sensors:
            sensor_key = f"sensor_{sensor['id']}"
            image, _ = self._image_reader.read(sensor_key)
            image_dict[sensor_key] = image
        return image_dict

    def wait_for_images(self, timeout=5.0):
        """Wait until all sensors have produced at least one frame."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            images = self.get_images()
            if all(img is not None for img in images.values()):
                return True
            time.sleep(0.1)
        return False

    def stop(self):
        """Drop one reference. 실제 StopSensors 는 마지막 caller 가 stop 할 때만 호출."""
        if self._session_id is None or self._cache_key is None:
            return
        should_stop = False
        with _CACHE_LOCK:
            entry = _SESSION_CACHE.get(self._cache_key)
            if entry is not None and entry['id'] == self._session_id:
                entry['refs'] -= 1
                if entry['refs'] <= 0:
                    _SESSION_CACHE.pop(self._cache_key, None)
                    should_stop = True
            else:
                # cache mismatch (이미 다른 lifecycle 에서 비워짐) — 그냥 RPC 보냄
                should_stop = True

        if should_stop:
            try:
                client = get_bridge_client()
                client.obs.StopSensors(pb.SensorSessionId(id=self._session_id))
                print(f"[RemoteObs] Sensor session stopped: id={self._session_id}", flush=True)
            except Exception as e:
                print(f"[RemoteObs] Error stopping sensors: {e}", flush=True)
        else:
            print(f"[RemoteObs] Released ref on session id={self._session_id} (still in use)", flush=True)

        self._session_id = None
        self._cache_key = None

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass
