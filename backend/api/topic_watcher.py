# -*- coding: utf-8 -*-
"""
Topic watcher — backend가 ros2_bridge gRPC `ListTopics`를 1Hz로 폴링하고,
변경이 감지되면 socketio로 모든 연결된 클라이언트에 broadcast 한다.

이로써 프론트엔드가 클라이언트당 / 컴포저블당 폴링하지 않아도 토픽 가시성을
실시간으로 알 수 있고, ros2 graph 쿼리는 백엔드 단일 워커가 1Hz만 발생.

Frontend 계약:
    Event 'topics_changed':
      payload = { topics: [{ name: str, type: str }, ...] }
    이벤트는 (1) 변경이 감지될 때마다, (2) 새 클라이언트가 connect할 때 그 클라이언트에게만 발행.
"""
import threading
import time
from typing import Optional


class TopicWatcher:
    POLL_INTERVAL_SEC = 1.0

    def __init__(self, socketio, bridge_client):
        self._socketio = socketio
        self._bridge_client = bridge_client
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        # 최신 스냅샷 — connect 이벤트에서 즉시 응답하기 위해 캐시.
        self._snapshot_lock = threading.Lock()
        self._snapshot: list = []  # list[{'name': str, 'type': str}]

    # --- public ---------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, name="TopicWatcher", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_snapshot(self) -> list:
        """Return a copy of the most recent topic list (safe to send to a single client)."""
        with self._snapshot_lock:
            return list(self._snapshot)

    # --- internal -------------------------------------------------------
    def _fetch(self) -> Optional[list]:
        """Call bridge gRPC ListTopics. Returns None on transient failure."""
        try:
            from ..bridge.generated import robot_bridge_pb2 as pb
            result = self._bridge_client.driver.ListTopics(pb.Empty())
            return [{'name': t.name, 'type': t.type} for t in result.topics]
        except Exception:
            return None

    @staticmethod
    def _names(topics: list) -> set:
        return {t['name'] for t in topics}

    def _loop(self) -> None:
        prev_names: set = set()
        while not self._stop_event.is_set():
            topics = self._fetch()
            if topics is None:
                # 브리지 일시 단절 — 스냅샷 유지, 다음 tick에 재시도.
                self._stop_event.wait(self.POLL_INTERVAL_SEC)
                continue

            curr_names = self._names(topics)
            with self._snapshot_lock:
                self._snapshot = topics

            if curr_names != prev_names:
                try:
                    self._socketio.emit('topics_changed', {'topics': topics})
                except Exception as e:
                    print(f"[TopicWatcher] emit failed: {e}")
                prev_names = curr_names

            self._stop_event.wait(self.POLL_INTERVAL_SEC)
