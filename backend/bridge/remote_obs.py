# -*- coding: utf-8 -*-
"""
RemoteObs — 센서 구독 gRPC proxy + shm 직접 읽기.

gRPC는 센서 구독 시작/종료(lifecycle)에만 사용.
이미지 데이터는 /dev/shm/easytrainer/에서 직접 읽음 (zero-copy).
"""
import json
import time

from .generated import robot_bridge_pb2 as pb
from .client import get_bridge_client
from .image_reader import ImageBridgeReader


class RemoteObs:
    def __init__(self, sensors):
        self.sensors = sensors
        self._image_reader = ImageBridgeReader()
        self._session_id = None

        if sensors:
            client = get_bridge_client()
            result = client.obs.StartSensors(
                pb.SensorConfig(sensors_json=json.dumps(sensors))
            )
            self._session_id = result.id
            print(f"[RemoteObs] Sensor session started: id={self._session_id}")

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
        """Stop sensor subscriptions via gRPC."""
        if self._session_id is not None and self._session_id > 0:
            try:
                client = get_bridge_client()
                client.obs.StopSensors(pb.SensorSessionId(id=self._session_id))
                print(f"[RemoteObs] Sensor session stopped: id={self._session_id}")
            except Exception as e:
                print(f"[RemoteObs] Error stopping sensors: {e}")
            self._session_id = None

    def __del__(self):
        self.stop()
