#!/usr/bin/env python3
import argparse
import os
import shutil
import sys
import tempfile


def main():
    ap = argparse.ArgumentParser(description="Run Orator migrations in an isolated temp copy of the DB folder")
    ap.add_argument(
        "--dbdir",
        default=os.path.join(os.getcwd(), "src", "backend", "database"),
        help="Path to the database folder containing config/, migrations/, and main.db"
    )
    args = ap.parse_args()

    src_dir = os.path.abspath(args.dbdir)
    if not os.path.isdir(src_dir):
        print(f"[ERR] DB directory not found: {src_dir}", file=sys.stderr)
        return 2

    tmp_root = tempfile.mkdtemp(prefix="orator_dbtest_")
    dst_dir = os.path.join(tmp_root, "database")
    shutil.copytree(src_dir, dst_dir, dirs_exist_ok=True)
    print(f"[TEST] Using temp copy: {dst_dir}")

    # Add temp dir to sys.path so `config.database` imports resolve
    sys.path.insert(0, dst_dir)

    try:
        import importlib
        import importlib.metadata as m
        print("[TEST] python:", sys.version.replace("\n", " "))
        for p in ["orator", "cleo", "inflection", "pyyaml", "faker", "pendulum", "backpack"]:
            try:
                print(f"[TEST] pkg {p}=", m.version(p))
            except Exception as e:
                print(f"[TEST] pkg {p}=n/a ({e})")

        cfg = importlib.import_module("config.database")
        if hasattr(cfg, "DATABASES"):
            connections = {k: v for k, v in cfg.DATABASES.items() if isinstance(v, dict)}
        elif hasattr(cfg, "config"):
            connections = cfg.config
        else:
            raise RuntimeError("No DATABASES/config mapping in config/database.py")

        from orator import DatabaseManager
        from orator.migrations import Migrator, DatabaseMigrationRepository

        db = DatabaseManager(connections)
        repo = DatabaseMigrationRepository(db, "migrations")
        if not repo.repository_exists():
            print("[TEST] creating migrations repository table ...")
            repo.create_repository()

        migrator = Migrator(repo, db)
        path = os.path.join(dst_dir, "migrations")
        print("[TEST] running migrations from", path)
        migrator.run(path)
        print("[TEST] migration complete")
        db_path = os.path.join(dst_dir, "main.db")
        if os.path.exists(db_path):
            print("[TEST] db size:", os.path.getsize(db_path))
        else:
            print("[TEST] db file missing:", db_path)
        return 0
    except Exception as e:
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

