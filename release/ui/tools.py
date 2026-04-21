from __future__ import annotations

import json
import os
import shutil
import sqlite3
import subprocess
import time
from pathlib import Path

from app_context import (
    APP_HOME,
    DATA_ROOT,
    REPO_ROOT_CANDIDATE,
    QFileDialog,
    QInputDialog,
    QMessageBox,
    QProcess,
    load_config,
    save_config,
)
from service import docker_compose_available

class ToolingMixin:
    def _set_import_choice(self, choice: str):
        if choice not in ("DB", "DATASET", "MODEL"):
            return
        self._pad_import_choice = choice
        self._update_pad_choice_labels()
        self._update_pad_choice_tooltips()
        self._style_choice_buttons(getattr(self, "pad_import_buttons", {}))

    def _set_export_choice(self, choice: str):
        if choice not in ("DB", "DATASET", "MODEL"):
            return
        self._pad_export_choice = choice
        self._update_pad_choice_labels()
        self._update_pad_choice_tooltips()
        self._style_choice_buttons(getattr(self, "pad_export_buttons", {}))

    def _is_valid_dev_src(self, path: Path | None) -> bool:
        try:
            return bool(path) and path.is_dir() and (path / "src" / "backend").exists() and (path / "src" / "ui").exists()
        except Exception:
            return False

    def _on_import_clicked(self):
        choice = self._current_choice_from_buttons(
            getattr(self, "pad_import_buttons", {}),
            self._pad_import_choice,
        )
        self._run_import_selected(choice)

    def _on_export_clicked(self):
        choice = self._current_choice_from_buttons(
            getattr(self, "pad_export_buttons", {}),
            self._pad_export_choice,
        )
        self._run_export_selected(choice)

    def _map_backend_db_path(self, raw_path: str) -> Path | None:
        try:
            path = Path(raw_path)
        except Exception:
            return None
        if not path.is_absolute():
            return None
        if path == Path("/root/src") or str(path).startswith("/root/src/"):
            try:
                rel = path.relative_to("/root/src")
            except Exception:
                return None
            for base in (getattr(self, "project_root", None), getattr(self, "dev_src_root", None)):
                if isinstance(base, Path):
                    return base / "src" / rel
        return path

    def _fetch_backend_db_path(self) -> Path | None:
        try:
            from urllib import request
            req = request.Request("http://127.0.0.1:5000/api/db/path", method="GET")
            with request.urlopen(req, timeout=0.8) as resp:
                payload = json.loads(resp.read().decode("utf-8") or "{}")
            raw_path = payload.get("path")
            if not raw_path:
                return None
            return self._map_backend_db_path(str(raw_path))
        except Exception:
            return None

    def _db_main_path(self) -> Path:
        data_db = DATA_ROOT / "database" / "main.db"
        backend_path = self._fetch_backend_db_path()
        if backend_path is not None:
            return backend_path
        running = False
        try:
            running = "service" in self._get_running_services()
        except Exception:
            running = False
        if running:
            return data_db
        project_under_data = False
        try:
            if isinstance(self.project_root, Path):
                pr = self.project_root.resolve()
                dr = DATA_ROOT.resolve()
                project_under_data = pr == dr or dr in pr.parents
        except Exception:
            project_under_data = False
        if self._is_valid_dev_src(self.dev_src_root) and not project_under_data:
            return self.dev_src_root / "src" / "backend" / "database" / "main.db"
        if self._is_valid_project_root(self.project_root) and not project_under_data:
            return self.project_root / "src" / "backend" / "database" / "main.db"
        if docker_compose_available() and self._is_valid_project_root(self.project_root):
            return data_db
        if self._is_valid_project_root(self.project_root):
            return self.project_root / "src" / "backend" / "database" / "main.db"
        return data_db

    def _cleanup_sqlite_sidecars(self, db_path: Path):
        for suffix in ("-wal", "-shm"):
            sidecar = db_path.with_name(db_path.name + suffix)
            try:
                if sidecar.exists():
                    sidecar.unlink()
            except Exception:
                pass

    def _copy_file_best_effort(self, src: Path, dest: Path, context: str):
        try:
            shutil.copy2(src, dest)
            return
        except OSError as e:
            try:
                shutil.copyfile(src, dest)
            except Exception:
                raise
            self.append_log(f"[{context}][INFO] 메타데이터 복사 실패로 기본 복사로 진행: {e}")

    def _unique_import_path(self, dest: Path) -> Path:
        if not dest.exists():
            return dest
        parent = dest.parent
        suffix = dest.suffix
        stem = dest.stem if suffix else dest.name
        for i in range(1, 1000):
            candidate = parent / f"{stem}_{i}{suffix}"
            if not candidate.exists():
                return candidate
        return parent / f"{stem}_{int(time.time())}{suffix}"

    def _copy_lerobot_dataset(self, src: Path, dest: Path):
        """LeRobot 데이터셋 디렉토리 구조를 복사한다.

        LeRobot 포맷: meta/, data/, images/ 디렉토리.
        레거시 HDF5 포맷: episode_*.hdf5 파일만 존재.
        자동으로 감지하여 적절히 복사한다.
        """
        # LeRobot 포맷 감지: meta/info.json 존재 여부
        is_lerobot = (src / "meta" / "info.json").is_file()

        if is_lerobot:
            # LeRobot 포맷: meta, data, images, videos 디렉토리를 재귀 복사
            for subdir in ["meta", "data", "images", "videos"]:
                src_sub = src / subdir
                if src_sub.is_dir():
                    dest_sub = dest / subdir
                    if dest_sub.exists():
                        shutil.rmtree(dest_sub)
                    shutil.copytree(src_sub, dest_sub)
            self.append_log(f"[IMPORT] LeRobot 데이터셋 복사 완료: {src.name}")
        else:
            # 레거시 HDF5 포맷: .hdf5 파일만 복사
            copied = 0
            for f in src.iterdir():
                if f.is_file() and f.suffix == '.hdf5':
                    shutil.copy2(f, dest / f.name)
                    copied += 1
            self.append_log(f"[IMPORT] HDF5 에피소드 {copied}개 복사 완료: {src.name}")

    def _reload_backend_db(self) -> bool:
        try:
            from urllib import request
            req = request.Request("http://127.0.0.1:5000/api/db/reload", method="POST")
            with request.urlopen(req, timeout=2.0) as resp:
                payload = json.loads(resp.read().decode("utf-8") or "{}")
            return payload.get("status") == "success"
        except Exception as e:
            self.append_log(f"[IMPORT][WARN] DB 갱신 요청 실패: {e}")
            return False

    def _resolve_backend_subdir(self, name: str) -> Path | None:
        roots: list[Path] = []
        root = getattr(self, "project_root", None)
        dev_root = getattr(self, "dev_src_root", None)
        prefer_dev = False
        project_under_data = False
        try:
            if isinstance(root, Path):
                pr = root.resolve()
                dr = DATA_ROOT.resolve()
                project_under_data = pr == dr or dr in pr.parents
        except Exception:
            project_under_data = False
        try:
            prefer_dev = ("service" not in self._get_running_services()) and not project_under_data
        except Exception:
            prefer_dev = False
        ordered_roots = [dev_root, root] if prefer_dev else [root, dev_root]
        for candidate in ordered_roots:
            if isinstance(candidate, Path) and candidate not in roots:
                roots.append(candidate)
        for base in roots:
            try:
                if base.exists():
                    return base / "src" / "backend" / name
            except Exception:
                continue
        return None

    def _db_table_columns(self, conn, table: str) -> set[str]:
        try:
            cur = conn.execute(f"PRAGMA table_info({table})")
            return {row[1] for row in cur.fetchall()}
        except Exception:
            return set()

    def _fetch_tasks_from_db(self) -> list[tuple[int, str]]:
        db_path = self._db_main_path()
        if not db_path.exists():
            return []
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            cur = conn.execute("SELECT id, name FROM tasks WHERE deleted_at IS NULL ORDER BY id")
            tasks = []
            for row in cur.fetchall():
                try:
                    tid = int(row[0])
                except Exception:
                    continue
                name = row[1] or f"Task {tid}"
                tasks.append((tid, name))
            return tasks
        except Exception:
            return []
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _select_task_id(self) -> int | None:
        tasks = self._fetch_tasks_from_db()
        if not tasks:
            return None
        if len(tasks) == 1:
            return tasks[0][0]
        items = [f"{tid}: {name}" for tid, name in tasks]
        choice, ok = QInputDialog.getItem(
            self,
            "작업 선택",
            "연결할 작업을 선택하세요.",
            items,
            0,
            False,
        )
        if not ok or not choice:
            return None
        try:
            return int(str(choice).split(":", 1)[0].strip())
        except Exception:
            return None

    def _fetch_policies_from_db(self) -> list[tuple[int, str]]:
        db_path = self._db_main_path()
        if not db_path.exists():
            return []
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            cur = conn.execute("SELECT id, name, type FROM policies WHERE deleted_at IS NULL ORDER BY id")
            policies = []
            for row in cur.fetchall():
                try:
                    pid = int(row[0])
                except Exception:
                    continue
                name = row[1] or f"Policy {pid}"
                ptype = row[2] or ""
                label = f"{name} ({ptype})" if ptype else name
                policies.append((pid, label))
            return policies
        except Exception:
            return []
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _select_policy_id(self) -> int | None:
        policies = self._fetch_policies_from_db()
        if not policies:
            return None
        if len(policies) == 1:
            return policies[0][0]
        items = [f"{pid}: {label}" for pid, label in policies]
        choice, ok = QInputDialog.getItem(
            self,
            "정책 선택",
            "연결할 정책을 선택하세요.",
            items,
            0,
            False,
        )
        if not ok or not choice:
            return None
        try:
            return int(str(choice).split(":", 1)[0].strip())
        except Exception:
            return None

    def _insert_dataset_record(self, name: str, task_id: int | None) -> int | None:
        db_path = self._db_main_path()
        if not db_path.exists():
            return None
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            columns = self._db_table_columns(conn, "datasets")
            if not columns:
                return None
            fields = []
            values = []
            if "name" in columns:
                fields.append("name")
                values.append(name)
            if "task_id" in columns:
                fields.append("task_id")
                values.append(task_id)
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            if "created_at" in columns:
                fields.append("created_at")
                values.append(timestamp)
            if "updated_at" in columns:
                fields.append("updated_at")
                values.append(timestamp)
            if "deleted_at" in columns:
                fields.append("deleted_at")
                values.append(None)
            if not fields:
                return None
            placeholders = ", ".join(["?"] * len(fields))
            sql = f"INSERT INTO datasets ({', '.join(fields)}) VALUES ({placeholders})"
            cur = conn.execute(sql, values)
            conn.commit()
            return cur.lastrowid
        except Exception:
            return None
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _insert_checkpoint_record(
        self,
        name: str,
        policy_id: int | None,
        task_id: int | None = None,
        is_base_model: bool = True,
    ) -> int | None:
        if policy_id is None:
            return None
        db_path = self._db_main_path()
        if not db_path.exists():
            return None
        conn = None
        try:
            conn = sqlite3.connect(str(db_path))
            columns = self._db_table_columns(conn, "checkpoints")
            if not columns:
                return None
            fields = []
            values = []
            if "name" in columns:
                fields.append("name")
                values.append(name)
            if "policy_id" in columns:
                fields.append("policy_id")
                values.append(policy_id)
            if "task_id" in columns:
                fields.append("task_id")
                values.append(task_id)
            if "dataset_info" in columns:
                fields.append("dataset_info")
                values.append(json.dumps({}))
            if "status" in columns:
                fields.append("status")
                values.append("finished")
            if "is_base_model" in columns:
                fields.append("is_base_model")
                values.append(1 if is_base_model else 0)
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            if "created_at" in columns:
                fields.append("created_at")
                values.append(timestamp)
            if "updated_at" in columns:
                fields.append("updated_at")
                values.append(timestamp)
            if "deleted_at" in columns:
                fields.append("deleted_at")
                values.append(None)
            if not fields:
                return None
            placeholders = ", ".join(["?"] * len(fields))
            sql = f"INSERT INTO checkpoints ({', '.join(fields)}) VALUES ({placeholders})"
            cur = conn.execute(sql, values)
            conn.commit()
            return cur.lastrowid
        except Exception:
            return None
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass

    def _run_import_selected(self, choice: str):
        try:
            if choice == "DB":
                result = QFileDialog.getOpenFileName(
                    self,
                    "DB 불러오기",
                    str(Path.home()),
                    "DB Files (*.db *.sqlite *.sqlite3);;All Files (*)",
                )
                src_path = result[0] if isinstance(result, (tuple, list)) else result
            else:
                src_path = QFileDialog.getExistingDirectory(
                    self,
                    f"{choice} 불러오기",
                    str(Path.home()),
                )
        except Exception:
            return
        if not src_path:
            return
        src = Path(src_path)
        try:
            if choice == "DB":
                dest = self._db_main_path()
                dest.parent.mkdir(parents=True, exist_ok=True)
                if src.is_dir():
                    raise ValueError("DB 파일이 디렉터리입니다.")
                self._cleanup_sqlite_sidecars(dest)
                self._copy_file_best_effort(src, dest, "IMPORT")
                self._cleanup_sqlite_sidecars(dest)
                self.append_log(f"[IMPORT] {choice} -> {dest}")
                if docker_compose_available() and self._is_valid_project_root(self.project_root):
                    running = False
                    try:
                        running = "service" in self._get_running_services()
                    except Exception:
                        running = False
                    if running:
                        if self._reload_backend_db():
                            self.append_log("[IMPORT] DB 연결을 갱신했습니다.")
                        else:
                            self.append_log("[IMPORT][WARN] DB 갱신에 실패했습니다. UI만 새로고침합니다.")
                    else:
                        self.append_log("[IMPORT][INFO] 서비스가 실행 중이 아니어서 UI만 새로고침합니다.")
                    self.load_ui(open_mode="CURRENT", force_refresh=True)
                else:
                    if self._reload_backend_db():
                        self.append_log("[IMPORT] DB 연결을 갱신했습니다.")
                    self.load_ui(open_mode="CURRENT", force_refresh=True)
                return
            elif choice == "DATASET":
                dest_root = self._resolve_backend_subdir("datasets")
                if dest_root is None:
                    self.append_log("[IMPORT][WARN] DATASET 경로를 찾을 수 없습니다.")
                    return
                dest_root.mkdir(parents=True, exist_ok=True)
                tasks = self._fetch_tasks_from_db()
                if not tasks:
                    self.append_log("[IMPORT][WARN] 작업이 없어 DATASET을 적용할 수 없습니다. 작업을 먼저 생성하세요.")
                    return
                task_id = tasks[0][0] if len(tasks) == 1 else self._select_task_id()
                if task_id is None:
                    self.append_log("[IMPORT][INFO] DATASET 불러오기를 취소했습니다.")
                    return
                dataset_name = src.stem if src.is_file() else src.name
                dataset_id = self._insert_dataset_record(dataset_name, task_id)
                if dataset_id is None:
                    self.append_log("[IMPORT][WARN] DB 등록 실패로 파일만 복사합니다.")
                    dest = self._unique_import_path(dest_root / src.name)
                else:
                    dest = dest_root / str(dataset_id)
                dest.mkdir(parents=True, exist_ok=True)
                if src.is_dir():
                    self._copy_lerobot_dataset(src, dest)
                else:
                    shutil.copy2(src, dest / src.name)
            elif choice == "MODEL":
                dest_root = self._resolve_backend_subdir("checkpoints") or self._resolve_backend_subdir("models")
                if dest_root is None:
                    self.append_log("[IMPORT][WARN] MODEL 경로를 찾을 수 없습니다.")
                    return
                dest_root.mkdir(parents=True, exist_ok=True)
                policies = self._fetch_policies_from_db()
                if not policies:
                    self.append_log("[IMPORT][WARN] 정책이 없어 MODEL을 적용할 수 없습니다. 정책을 먼저 생성하세요.")
                    return
                policy_id = policies[0][0] if len(policies) == 1 else self._select_policy_id()
                if policy_id is None:
                    self.append_log("[IMPORT][INFO] MODEL 불러오기를 취소했습니다.")
                    return
                tasks = self._fetch_tasks_from_db()
                task_id = None
                if tasks:
                    task_id = tasks[0][0] if len(tasks) == 1 else self._select_task_id()
                is_base_model = task_id is None
                model_name = src.stem if src.is_file() else src.name
                checkpoint_id = self._insert_checkpoint_record(
                    model_name,
                    policy_id,
                    task_id=task_id,
                    is_base_model=is_base_model,
                )
                if checkpoint_id is None:
                    self.append_log("[IMPORT][WARN] DB 등록 실패로 파일만 복사합니다.")
                    dest = self._unique_import_path(dest_root / src.name)
                    if src.is_dir():
                        shutil.copytree(src, dest, dirs_exist_ok=True)
                    else:
                        shutil.copy2(src, dest)
                else:
                    dest = dest_root / str(checkpoint_id)
                    if src.is_dir():
                        shutil.copytree(src, dest, dirs_exist_ok=True)
                    else:
                        dest.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(src, dest / src.name)
            else:
                return
            self.append_log(f"[IMPORT] {choice} -> {dest}")
            self.load_ui(open_mode="CURRENT", force_refresh=True)
        except Exception as e:
            self.append_log(f"[IMPORT][ERROR] {choice} 불러오기 실패: {e}")

    def _resolve_export_source(self, choice: str) -> Path | None:
        if choice == "DB":
            db_path = self._db_main_path()
            try:
                return db_path if db_path.exists() else None
            except Exception:
                return None
        if choice == "DATASET":
            root = self._resolve_backend_subdir("datasets")
            try:
                return root if root and root.exists() else None
            except Exception:
                return None
        if choice == "MODEL":
            for name in ("checkpoints", "models"):
                root = self._resolve_backend_subdir(name)
                try:
                    if root and root.exists():
                        return root
                except Exception:
                    continue
        return None

    def _run_export_selected(self, choice: str):
        src = self._resolve_export_source(choice)
        if not src:
            self.append_log(f"[EXPORT][WARN] {choice} 내보낼 항목을 찾지 못했습니다.")
            return
        try:
            if choice == "DB":
                default_name = src.name if src.name else "main.db"
                result = QFileDialog.getSaveFileName(
                    self,
                    "DB 저장 위치 선택",
                    str(Path.home() / default_name),
                    "DB Files (*.db *.sqlite *.sqlite3);;All Files (*)",
                )
                dest_path = result[0] if isinstance(result, (tuple, list)) else result
                if not dest_path:
                    return
                dest = Path(dest_path)
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest)
            else:
                dest_dir = QFileDialog.getExistingDirectory(
                    self,
                    f"{choice} 저장 위치 선택",
                    str(Path.home()),
                )
                if not dest_dir:
                    return
                dest = Path(dest_dir) / src.name
                if src.is_dir():
                    shutil.copytree(src, dest, dirs_exist_ok=True)
                else:
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, dest)
            self.append_log(f"[EXPORT] {choice} -> {dest}")
        except Exception as e:
            self.append_log(f"[EXPORT][ERROR] {choice} 내보내기 실패: {e}")

    def _prompt_dev_src_then_apply(self):
        if self._is_valid_dev_src(self.dev_src_root):
            return
        self.on_select_dev_src()
        if self._is_valid_dev_src(self.dev_src_root):
            self._quick_apply_from_dev_src()

    def on_select_dev_src(self):
        folder = QFileDialog.getExistingDirectory(self, "원본 프로젝트 루트 선택 (src/backend, src/ui 포함)", str(self.dev_src_root or Path.home()))
        if not folder:
            return
        path = Path(folder)
        if not self._is_valid_dev_src(path):
            QMessageBox.warning(self, "잘못된 경로", "선택한 폴더에 src/backend 또는 src/ui가 없습니다.")
            return
        self.dev_src_root = path
        cfg = load_config()
        cfg["dev_src_root"] = str(self.dev_src_root)
        save_config(cfg)
        self.update_dev_src_label()
        self.update_buttons()

    def on_apply_and_restart(self):
        self.append_log("[SYNC] 적용 중: backend/ui/compose 복사...")
        if not self._sync_dev_files():
            return
        if not self._apply_compose_variant(self.install_variant):
            QMessageBox.critical(self, "오류", "선택한 설치 옵션에 맞는 docker-compose 템플릿을 찾을 수 없습니다.")
            return
        self.append_log("[SYNC] 완료. 서비스 재시작 중...")
        self._show_preload_dialog("Easy Trainer 준비중...")
        def _after_restart(exit_code: int, *_):
            self.append_log("[RESTART] 완료")
            if exit_code == 0:
                self._wait_for_services_ready(self.load_ui)
            else:
                self._hide_preload_dialog()
        self._clear_conflicting_containers()
        self._ensure_service_running("RESTART", restart_if_running=True, on_finish=_after_restart)

    def _quick_apply_script_candidates(self) -> list[Path]:
        return [
            REPO_ROOT_CANDIDATE / "scripts" / "quick_apply.sh",
            Path("/opt/easytrainer/scripts/quick_apply.sh"),
            APP_HOME / "scripts" / "quick_apply.sh",
            self.project_root / "scripts" / "quick_apply.sh",
        ]

    def _resolve_quick_apply_script(self) -> Path | None:
        for cand in self._quick_apply_script_candidates():
            try:
                if cand.is_file() and os.access(cand, os.X_OK):
                    return cand
            except Exception:
                continue
        return None

    def _sync_dirs(self, src: Path, dst: Path):
        dst.mkdir(parents=True, exist_ok=True)
        for item in src.rglob("*"):
            rel = item.relative_to(src)
            target = dst / rel
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(item, target)

    def _sync_core_files(self):
        if not self.dev_src_root:
            return
        files = [
            "docker-compose.yml",
            "docker-compose.dev.yml",
            "docker-compose.cpu.yml",
            "docker-compose.gpu.yml",
            "start_services.sh",
            "src/kill.sh",
            "Dockerfile",
            ".dockerignore",
            "requirements.txt",
            "requirements.min.txt",
        ]
        for name in files:
            src = self.dev_src_root / name
            if not src.exists():
                continue
            dst = self.project_root / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)

    def _sync_release_ui(self):
        if not self.dev_src_root:
            return
        src = self.dev_src_root / "release" / "ui"
        if not src.is_dir():
            return
        dests: list[Path] = []
        install_root = None
        try:
            install_root = getattr(self.project_root, "parent", None)
        except Exception:
            install_root = None
        if install_root and (install_root / "ui").is_dir():
            dests.append(install_root / "ui")
        system_ui = Path("/opt/easytrainer/ui")
        if system_ui.is_dir() and system_ui not in dests:
            dests.append(system_ui)
        if not dests:
            return
        for dst in dests:
            try:
                shutil.copytree(
                    src,
                    dst,
                    dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
                )
                self.append_log(f"[SYNC] UI 업데이트: {dst}")
            except Exception as e:
                self.append_log(f"[SYNC][WARN] UI 업데이트 실패: {dst} ({e})")

    def _sync_dev_files(self, show_errors: bool = True) -> bool:
        if not self._is_valid_project_root(self.project_root):
            if show_errors:
                QMessageBox.warning(self, "프로젝트 필요", "프로젝트가 아직 준비되지 않았습니다.")
            return False
        if not self._is_valid_dev_src(self.dev_src_root):
            if show_errors:
                QMessageBox.warning(self, "원본 필요", "원본 프로젝트 경로를 먼저 선택하세요.")
            return False
        script = self._resolve_quick_apply_script()
        if script:
            try:
                self.append_log(f"[SYNC] quick_apply 실행: {script}")
            except Exception:
                pass
            try:
                result = subprocess.run(
                    [str(script), str(self.dev_src_root), str(self.project_root)],
                    capture_output=True,
                    text=True,
                )
            except Exception as e:
                if show_errors:
                    QMessageBox.critical(self, "오류", f"동기화 스크립트 실행 실패: {e}")
                else:
                    self.append_log(f"[SYNC][ERROR] 스크립트 실행 실패: {e}")
                return False
            for stream_label, payload in (("OUT", result.stdout), ("ERR", result.stderr)):
                if not payload:
                    continue
                for line in payload.splitlines():
                    self.append_log(f"[SYNC][{stream_label}] {line}")
            if result.returncode == 0:
                self._apply_compose_variant(self.install_variant)
                return True
            if show_errors:
                QMessageBox.critical(self, "오류", "[SYNC] quick_apply 스크립트가 실패했습니다. 로그를 확인하세요.")
            else:
                self.append_log("[SYNC][ERROR] quick_apply 스크립트가 실패했습니다.")
            return False
        # Fallback to in-app copy when script is missing (legacy behaviour)
        try:
            self.append_log("[SYNC][WARN] quick_apply 스크립트를 찾을 수 없어 내부 복사를 사용합니다.")
            self._sync_dirs(self.dev_src_root / "src" / "backend", self.project_root / "src" / "backend")
            self._sync_dirs(self.dev_src_root / "src" / "ui", self.project_root / "src" / "ui")
            self._sync_dirs(self.dev_src_root / "ros2" / "ros2_ws" / "src", self.project_root / "ros2" / "ros2_ws" / "src")
            self._sync_release_ui()
            self._sync_core_files()
            self._apply_compose_variant(self.install_variant)
            return True
        except Exception as e:
            if show_errors:
                QMessageBox.critical(self, "오류", f"동기화 실패: {e}")
            else:
                self.append_log(f"[SYNC][ERROR] {e}")
            return False

    def _auto_sync_on_start(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            try:
                self.append_log("[SYNC][AUTO] 원본 경로가 없어 자동 동기화를 건너뜁니다.")
            except Exception:
                pass
            return
        try:
            self.append_log("[SYNC][AUTO] 최신 코드 자동 동기화를 시작합니다.")
        except Exception:
            pass
        if not self._sync_dev_files(show_errors=False):
            try:
                self.append_log("[SYNC][AUTO][WARN] 자동 동기화에 실패했습니다.")
            except Exception:
                pass
            return
        try:
            self.append_log("[SYNC][AUTO] 자동 동기화 완료.")
        except Exception:
            pass

    def _quick_apply_from_dev_src(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            self.load_ui(open_mode="CURRENT")
            return
        compose_ready = docker_compose_available() and self._is_valid_project_root(self.project_root)
        fast_backend_reload = os.environ.get("EC_BACKEND_AUTORELOAD", "1") != "0"
        running = "service" in self._get_running_services() if compose_ready else False
        process_busy = self.process is not None and self.process.state() != QProcess.NotRunning
        cleanup_nodes = os.environ.get("EC_QUICK_SYNC_CLEANUP", "1") != "0"
        self.append_log("[SYNC] 원본에서 빠른 적용 중...")
        if compose_ready and running and fast_backend_reload and cleanup_nodes and not process_busy:
            self.append_log("[SYNC] ROS/로봇 노드 정리 중...")
            try:
                self._run_backend_kill(keep_backend=True, label="SYNC")
            except Exception:
                pass
        if not self._sync_dev_files(show_errors=False):
            self.append_log("[SYNC][WARN] 원본 적용에 실패했습니다. 경로를 확인하세요.")
            return
        # Default to fast path: assume backend autoreload is on (container default) to avoid restarts
        if compose_ready:
            running = "service" in self._get_running_services()
            if self.process is not None and self.process.state() != QProcess.NotRunning:
                if self._pending_quick_apply:
                    self.append_log("[SYNC][INFO] 다른 작업이 끝나면 대기 중인 빠른 동기화를 실행합니다.")
                    return
                self._pending_quick_apply = True
                self.append_log("[SYNC][INFO] 다른 작업 종료 후 빠른 동기화를 바로 실행합니다.")
                def _after_current(*_):
                    try:
                        self.process.finished.disconnect(_after_current)
                    except Exception:
                        pass
                    self._pending_quick_apply = False
                    self._quick_apply_from_dev_src()
                self.process.finished.connect(_after_current)
                return
            if running:
                # ROS2 컨테이너 재시작 (gRPC 서버는 autoreload 없으므로 재시작 필요)
                if "ros2" in self._get_running_services():
                    self.append_log("[SYNC] ROS2 컨테이너 재시작 중...")
                    try:
                        subprocess.run(
                            ["docker", "restart", "easy_collector_ros2"],
                            capture_output=True, text=True, timeout=30,
                        )
                        self.append_log("[SYNC] ROS2 컨테이너 재시작 완료.")
                    except Exception as e:
                        self.append_log(f"[SYNC][WARN] ROS2 재시작 실패: {e}")
                if fast_backend_reload:
                    self.append_log("[SYNC] 빠른 적용 완료. 메인 컨테이너는 autoreload/HMR로 반영, ROS2는 재시작됨.")
                    self.load_ui(open_mode="CURRENT")
                    return
                self.append_log("[SYNC][INFO] 컨테이너 유지 (autreload=off). 필요하면 수동 재시작해 주세요.")
                self.load_ui(open_mode="CURRENT")
                return
            self.append_log("[SYNC] 빠른 적용 완료. 서비스 시작 중...")
            self._show_preload_dialog("서비스 시작 중...")
            self._clear_conflicting_containers()
            def _after_restart(exit_code: int, *_):
                if exit_code == 0:
                    self.append_log("[START] 완료, UI 새로고침 중...")
                    self._wait_for_services_ready(lambda: self.load_ui(open_mode="CURRENT"))
                else:
                    self.append_log(f"[START][ERROR] 시작 실패 (code={exit_code})")
                    self._hide_preload_dialog()
            self._ensure_service_running("SYNC", restart_if_running=False, on_finish=_after_restart)
            return
        self.append_log("[SYNC] 빠른 적용 완료. UI 새로고침 중...")
        self.load_ui(open_mode="CURRENT")

    def _on_quick_apply_clicked(self):
        if not self._is_valid_dev_src(self.dev_src_root):
            self._show_pad_notice(1, "프로젝트 경로 지정", color="#ff6b6b", duration_ms=1000)
            try:
                self._dev_src_prompt_timer.stop()
            except Exception:
                pass
            self._dev_src_prompt_timer.start(1000)
            return
        self._quick_apply_from_dev_src()
