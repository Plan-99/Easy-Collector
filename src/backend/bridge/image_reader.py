# -*- coding: utf-8 -*-
"""
Shared Memory Image Reader.
메인 컨테이너에서 ROS2 컨테이너가 기록한 이미지를 zero-copy로 읽는다.
"""
import os
import struct
import numpy as np

SHM_BASE = '/dev/shm/easytrainer'

# Header format: width(I), height(I), channels(I), timestamp(d) = 20 bytes
HEADER_FMT = '<IIId'
HEADER_SIZE = struct.calcsize(HEADER_FMT)


class ImageBridgeReader:
    """공유 메모리에서 이미지 읽기."""

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
