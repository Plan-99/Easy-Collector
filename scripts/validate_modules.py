#!/usr/bin/env python3
"""Validate modules/**/module.json files.

Two modes:
  1. Schema check (always):  validate each module.json passed on argv.
  2. Version-bump check (--check-version-bump): for every module.json in argv,
     compare with HEAD; if any sibling file changed but `version` did not,
     fail. Pre-commit only stages files, so we read git index for siblings.

Exit codes:
  0 = all good, non-zero = at least one violation (each violation printed).
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

REQUIRED_TOP_LEVEL = {"id", "name", "version", "category", "description", "install", "dependencies", "check"}
ALLOWED_CATEGORIES = {"robot", "sensor", "extension"}
SEMVER_RE = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?$")
ID_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def _git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=REPO_ROOT, text=True).strip()


def _check_schema(path: Path) -> list[str]:
    errs: list[str] = []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        return [f"{path}: invalid JSON ({e})"]

    missing = REQUIRED_TOP_LEVEL - set(data.keys())
    if missing:
        errs.append(f"{path}: missing required keys: {sorted(missing)}")

    if "version" in data and not SEMVER_RE.match(str(data["version"])):
        errs.append(f"{path}: version '{data['version']}' is not semver X.Y.Z")

    if "category" in data and data["category"] not in ALLOWED_CATEGORIES:
        errs.append(
            f"{path}: category '{data['category']}' not in {sorted(ALLOWED_CATEGORIES)}"
        )

    if "id" in data and not ID_RE.match(str(data["id"])):
        errs.append(f"{path}: id '{data['id']}' must be snake_case (lowercase, [a-z0-9_], must start with a letter)")

    if "dependencies" in data and not isinstance(data["dependencies"], dict):
        errs.append(f"{path}: dependencies must be an object")

    return errs


def _module_dir_for(module_json: Path) -> Path:
    return module_json.parent


def _has_changes_other_than_module_json(module_dir: Path) -> bool:
    """Return True if any file other than module.json changed inside module_dir
    compared to HEAD (staged + unstaged)."""
    try:
        # Status of every path inside the module dir, including untracked.
        out = subprocess.check_output(
            [
                "git",
                "status",
                "--porcelain=v1",
                "--",
                str(module_dir.relative_to(REPO_ROOT)),
            ],
            cwd=REPO_ROOT,
            text=True,
        )
    except subprocess.CalledProcessError:
        return False
    for line in out.splitlines():
        if not line.strip():
            continue
        # Lines look like "XY path"; path may be quoted/renamed.
        path = line[3:].strip().split(" -> ")[-1].strip().strip('"')
        if path.endswith("/module.json") or path == "module.json":
            continue
        return True
    return False


def _version_at_head(module_json: Path) -> str | None:
    rel = module_json.relative_to(REPO_ROOT)
    try:
        blob = _git("show", f"HEAD:{rel.as_posix()}")
    except subprocess.CalledProcessError:
        return None  # new file
    try:
        return json.loads(blob).get("version")
    except json.JSONDecodeError:
        return None


def _check_version_bump(module_json: Path) -> list[str]:
    """If sibling files changed, module.json version must differ from HEAD."""
    module_dir = _module_dir_for(module_json)
    if not _has_changes_other_than_module_json(module_dir):
        return []
    try:
        current = json.loads(module_json.read_text(encoding="utf-8")).get("version")
    except (OSError, json.JSONDecodeError):
        return []
    head_version = _version_at_head(module_json)
    if head_version is None:
        return []  # brand new module — no bump to enforce
    if current == head_version:
        return [
            f"{module_json}: sibling files changed but version stayed at '{current}' — "
            "bump version (same version is skipped by modules-release CI)"
        ]
    return []


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*", help="module.json files (and others; ignored)")
    parser.add_argument("--check-version-bump", action="store_true")
    args = parser.parse_args()

    targets: list[Path] = []
    for p in args.paths:
        path = Path(p).resolve()
        if path.name == "module.json" and path.is_file():
            targets.append(path)

    if not targets:
        return 0

    errs: list[str] = []
    for path in targets:
        errs += _check_schema(path)
        if args.check_version_bump:
            errs += _check_version_bump(path)

    for e in errs:
        print(e, file=sys.stderr)
    return 1 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
