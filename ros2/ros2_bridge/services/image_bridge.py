# -*- coding: utf-8 -*-
"""
Shared Memory Image Bridge.
ROS2 센서 이미지를 /dev/shm/easytrainer/에 기록하여
메인 컨테이너에서 zero-copy로 읽을 수 있게 한다.
"""
import os
import struct
import threading
import numpy as np

SHM_BASE = '/dev/shm/easytrainer'

# Header format: width(I), height(I), channels(I), timestamp(d) = 20 bytes
HEADER_FMT = '<IIId'
HEADER_SIZE = struct.calcsize(HEADER_FMT)


class ImageBridgeWriter:
    """ROS2 컨테이너에서 이미지를 공유 메모리에 기록."""

    def __init__(self):
        os.makedirs(SHM_BASE, exist_ok=True)
        self._files = {}  # sensor_key -> file object
        self._lock = threading.Lock()

    def write(self, sensor_key: str, image: np.ndarray, timestamp: float = 0.0):
        """numpy 이미지를 공유 메모리 파일에 기록."""
        if image is None:
            return

        h, w = image.shape[:2]
        c = image.shape[2] if image.ndim == 3 else 1
        header = struct.pack(HEADER_FMT, w, h, c, timestamp)
        data = image.tobytes()

        path = os.path.join(SHM_BASE, sensor_key)
        total_size = HEADER_SIZE + len(data)

        with self._lock:
            f = self._files.get(sensor_key)
            if f is None:
                f = open(path, 'wb')
                self._files[sensor_key] = f

            f.seek(0)
            f.write(header + data)
            f.flush()

    def remove(self, sensor_key: str):
        """Remove a single sensor's shm file."""
        with self._lock:
            f = self._files.pop(sensor_key, None)
            if f is not None:
                try:
                    f.close()
                except Exception:
                    pass
        path = os.path.join(SHM_BASE, sensor_key)
        try:
            os.remove(path)
        except FileNotFoundError:
            pass

    def cleanup(self):
        with self._lock:
            for f in self._files.values():
                try:
                    f.close()
                except Exception:
                    pass
            self._files.clear()

        # Remove shm files
        try:
            for fname in os.listdir(SHM_BASE):
                os.remove(os.path.join(SHM_BASE, fname))
        except Exception:
            pass


class ImageBridgeReader:
    """메인 컨테이너에서 공유 메모리 이미지를 읽기."""

    def __init__(self):
        pass

    def read(self, sensor_key: str):
        """공유 메모리에서 이미지를 읽어 numpy 배열로 반환.

        Returns:
            tuple: (image: np.ndarray or None, timestamp: float)
        """
        path = os.path.join(SHM_BASE, sensor_key)
        try:
            with open(path, 'rb') as f:
                header = f.read(HEADER_SIZE)
                if len(header) < HEADER_SIZE:
                    return None, 0.0

                w, h, c, timestamp = struct.unpack(HEADER_FMT, header)
                expected_size = w * h * c
                data = f.read(expected_size)

                if len(data) < expected_size:
                    return None, 0.0

                if c == 1:
                    image = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
                else:
                    image = np.frombuffer(data, dtype=np.uint8).reshape(h, w, c)

                return image, timestamp
        except FileNotFoundError:
            return None, 0.0
        except Exception as e:
            print(f"[ImageBridgeReader] Error reading {sensor_key}: {e}")
            return None, 0.0
