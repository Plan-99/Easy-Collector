import json
import os
import shutil
import tempfile
import zipfile
from datetime import datetime, timezone

from flask import Blueprint, after_this_request, request, send_file
from werkzeug.utils import secure_filename

from ...configs.global_configs import DATASET_DIR
from ...database.config import database as db_config
from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
from ...database.models.dataset_model import Dataset as DatasetModel

# Shared paths for persistence
DATA_ROOT = os.environ.get("EASYTRAINER_DATA_DIR", "/opt/easytrainer")
CONFIG_PATH = os.environ.get(
    "EASYTRAINER_CONFIG_PATH", os.path.join(DATA_ROOT, "config.json")
)
DB_PATH = getattr(db_config, "DB_PATH", None)
CHECKPOINT_DIR = "/root/src/backend/checkpoints"

transfer_bp = Blueprint("transfer_bp", __name__)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _cleanup_tempdir(path: str):
    """Best-effort cleanup of temporary folders."""
    try:
        shutil.rmtree(path)
    except Exception:
        pass


@transfer_bp.route("/export/settings", methods=["GET"])
def export_settings():
    if not DB_PATH or not os.path.exists(DB_PATH):
        return {"status": "error", "message": "DB file not found"}, 404
    return send_file(
        DB_PATH,
        as_attachment=True,
        download_name="main.db",
        mimetype="application/octet-stream",
    )


@transfer_bp.route("/import/settings", methods=["POST"])
def import_settings():
    file = request.files.get("file")
    if not file:
        return {"status": "error", "message": "No file uploaded"}, 400

    if not DB_PATH:
        return {"status": "error", "message": "DB path is not configured"}, 500

    filename = secure_filename(file.filename or "")
    if not filename.lower().endswith(".db"):
        return {"status": "error", "message": "Only .db files are accepted"}, 400

    tmpdir = tempfile.mkdtemp(prefix="import_settings_")
    archive_path = os.path.join(tmpdir, filename or "main.db")
    file.save(archive_path)

    try:
        dest_dir = os.path.dirname(DB_PATH) or "/"
        os.makedirs(dest_dir, exist_ok=True)
        shutil.copy2(archive_path, DB_PATH)
    except Exception as exc:  # pylint: disable=broad-except
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": str(exc)}, 500

    _cleanup_tempdir(tmpdir)
    return {"status": "success", "message": "DB imported. Service restart may be required."}, 200


@transfer_bp.route("/export/dataset/<int:dataset_id>", methods=["GET"])
def export_dataset(dataset_id: int):
    dataset = DatasetModel.find(dataset_id)
    if not dataset:
        return {"status": "error", "message": "Dataset not found"}, 404

    dataset_dir = os.path.join(DATASET_DIR, str(dataset_id))
    if not os.path.isdir(dataset_dir):
        return {"status": "error", "message": "Dataset folder not found"}, 404

    tmpdir = tempfile.mkdtemp(prefix="export_dataset_")
    zip_path = os.path.join(tmpdir, f"dataset_{dataset_id}.zip")

    meta = dataset.to_dict()
    meta["exported_at"] = _utc_now()

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("metadata.json", json.dumps(meta, indent=2, ensure_ascii=False))
        for root, _, files in os.walk(dataset_dir):
            for fname in files:
                full_path = os.path.join(root, fname)
                rel_path = os.path.relpath(full_path, dataset_dir)
                zf.write(full_path, arcname=os.path.join("data", rel_path))

    @after_this_request
    def _cleanup(response):  # type: ignore
        _cleanup_tempdir(tmpdir)
        return response

    safe_name = f"dataset_{dataset_id}_{secure_filename(str(dataset.name or 'export'))}.zip"
    return send_file(
        zip_path,
        as_attachment=True,
        download_name=safe_name,
        mimetype="application/zip",
    )


def _load_json_field(raw_value, fallback=None):
    if raw_value is None:
        return fallback
    if isinstance(raw_value, (dict, list)):
        return raw_value
    try:
        return json.loads(raw_value)
    except Exception:  # pylint: disable=broad-except
        return fallback


@transfer_bp.route("/import/dataset", methods=["POST"])
def import_dataset():
    file = request.files.get("file")
    if not file:
        return {"status": "error", "message": "No file uploaded"}, 400

    mode = request.form.get("mode", "add")
    target_dataset_id = request.form.get("dataset_id")
    target_name = request.form.get("name")
    target_task_id = request.form.get("task_id")

    tmpdir = tempfile.mkdtemp(prefix="import_dataset_")
    archive_path = os.path.join(tmpdir, secure_filename(file.filename))
    file.save(archive_path)

    metadata = {}
    try:
        with zipfile.ZipFile(archive_path, "r") as zf:
            zf.extractall(tmpdir)
            meta_path = os.path.join(tmpdir, "metadata.json")
            if os.path.exists(meta_path):
                with open(meta_path, "r", encoding="utf-8") as f:
                    metadata = json.load(f)
    except zipfile.BadZipFile:
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": "Invalid zip file"}, 400

    # Resolve dataset attributes
    dataset_name = target_name or metadata.get("name") or "imported_dataset"
    dataset_task_id = target_task_id or metadata.get("task_id")
    try:
        dataset_task_id = int(dataset_task_id) if dataset_task_id is not None else None
    except Exception:
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": "task_id must be an integer"}, 400

    dataset = None
    if mode == "replace" and target_dataset_id:
        try:
            target_dataset_id = int(target_dataset_id)
        except Exception:
            _cleanup_tempdir(tmpdir)
            return {"status": "error", "message": "dataset_id must be an integer"}, 400
        dataset = DatasetModel.find(target_dataset_id)
        if not dataset:
            _cleanup_tempdir(tmpdir)
            return {"status": "error", "message": "Target dataset not found"}, 404
        if dataset_task_id is None:
            dataset_task_id = dataset.task_id
        dataset.name = dataset_name
        dataset.task_id = dataset_task_id
        dataset.save()
    else:
        if dataset_task_id is None:
            _cleanup_tempdir(tmpdir)
            return {"status": "error", "message": "task_id is required to import a dataset"}, 400
        dataset = DatasetModel.create(
            name=dataset_name,
            task_id=dataset_task_id,
        )

    dest_dir = os.path.join(DATASET_DIR, str(dataset.id))
    if os.path.exists(dest_dir):
        shutil.rmtree(dest_dir)
    os.makedirs(dest_dir, exist_ok=True)

    data_root = os.path.join(tmpdir, "data")
    if not os.path.isdir(data_root):
        data_root = tmpdir

    for root, _, files in os.walk(data_root):
        for fname in files:
            src_path = os.path.join(root, fname)
            if os.path.basename(src_path) == "metadata.json":
                continue
            rel_path = os.path.relpath(src_path, data_root)
            dest_path = os.path.join(dest_dir, rel_path)
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            shutil.copy2(src_path, dest_path)

    _cleanup_tempdir(tmpdir)
    return {
        "status": "success",
        "message": "Dataset imported",
        "dataset_id": dataset.id,
    }, 200


@transfer_bp.route("/export/checkpoint/<int:checkpoint_id>", methods=["GET"])
def export_checkpoint(checkpoint_id: int):
    checkpoint = CheckpointModel.find(checkpoint_id)
    if not checkpoint:
        return {"status": "error", "message": "Checkpoint not found"}, 404

    checkpoint_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint_id))
    if not os.path.isdir(checkpoint_dir):
        return {"status": "error", "message": "Checkpoint folder not found"}, 404

    tmpdir = tempfile.mkdtemp(prefix="export_checkpoint_")
    zip_path = os.path.join(tmpdir, f"checkpoint_{checkpoint_id}.zip")

    meta = checkpoint.to_dict()
    meta["exported_at"] = _utc_now()

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("metadata.json", json.dumps(meta, indent=2, ensure_ascii=False))
        for root, _, files in os.walk(checkpoint_dir):
            for fname in files:
                full_path = os.path.join(root, fname)
                rel_path = os.path.relpath(full_path, checkpoint_dir)
                zf.write(full_path, arcname=os.path.join("data", rel_path))

    @request.after_this_request
    def _cleanup(response):  # type: ignore
        _cleanup_tempdir(tmpdir)
        return response

    safe_name = f"checkpoint_{checkpoint_id}_{secure_filename(str(checkpoint.name or 'export'))}.zip"
    return send_file(
        zip_path,
        as_attachment=True,
        download_name=safe_name,
        mimetype="application/zip",
    )


@transfer_bp.route("/import/checkpoint", methods=["POST"])
def import_checkpoint():
    file = request.files.get("file")
    if not file:
        return {"status": "error", "message": "No file uploaded"}, 400

    mode = request.form.get("mode", "add")
    target_checkpoint_id = request.form.get("checkpoint_id")
    target_name = request.form.get("name")
    override_policy_id = request.form.get("policy_id")
    override_task_id = request.form.get("task_id")
    override_dataset_info = _load_json_field(request.form.get("dataset_info"))
    override_train_settings = _load_json_field(request.form.get("train_settings"))

    tmpdir = tempfile.mkdtemp(prefix="import_checkpoint_")
    archive_path = os.path.join(tmpdir, secure_filename(file.filename))
    file.save(archive_path)

    metadata = {}
    try:
        with zipfile.ZipFile(archive_path, "r") as zf:
            zf.extractall(tmpdir)
            meta_path = os.path.join(tmpdir, "metadata.json")
            if os.path.exists(meta_path):
                with open(meta_path, "r", encoding="utf-8") as f:
                    metadata = json.load(f)
    except zipfile.BadZipFile:
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": "Invalid zip file"}, 400

    base_policy_id = override_policy_id or metadata.get("policy_id")
    try:
        base_policy_id = int(base_policy_id) if base_policy_id is not None else None
    except Exception:
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": "policy_id must be an integer"}, 400
    if base_policy_id is None:
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": "policy_id is required to import a checkpoint"}, 400

    checkpoint_name = target_name or metadata.get("name") or "imported_checkpoint"
    dataset_info = override_dataset_info or metadata.get("dataset_info") or {}
    train_settings = override_train_settings or metadata.get("train_settings") or {}

    task_id_value = override_task_id or metadata.get("task_id")
    try:
        task_id_value = int(task_id_value) if task_id_value is not None else None
    except Exception:
        _cleanup_tempdir(tmpdir)
        return {"status": "error", "message": "task_id must be an integer"}, 400

    checkpoint = None
    if mode == "replace" and target_checkpoint_id:
        try:
            target_checkpoint_id = int(target_checkpoint_id)
        except Exception:
            _cleanup_tempdir(tmpdir)
            return {"status": "error", "message": "checkpoint_id must be an integer"}, 400
        checkpoint = CheckpointModel.find(target_checkpoint_id)
        if not checkpoint:
            _cleanup_tempdir(tmpdir)
            return {"status": "error", "message": "Target checkpoint not found"}, 404
        checkpoint.name = checkpoint_name
        checkpoint.policy_id = base_policy_id
        checkpoint.task_id = task_id_value
        checkpoint.dataset_info = dataset_info
        checkpoint.train_settings = train_settings
        checkpoint.loss = metadata.get("loss", checkpoint.loss)
        checkpoint.best_epoch = metadata.get("best_epoch", checkpoint.best_epoch)
        checkpoint.status = metadata.get("status", checkpoint.status or "finished")
        checkpoint.load_model_id = metadata.get("load_model_id", checkpoint.load_model_id)
        checkpoint.is_base_model = metadata.get("is_base_model", getattr(checkpoint, "is_base_model", False))
        checkpoint.save()
    else:
        checkpoint = CheckpointModel.create(
            name=checkpoint_name,
            task_id=task_id_value,
            policy_id=base_policy_id,
            dataset_info=dataset_info,
            train_settings=train_settings,
            status=metadata.get("status", "finished"),
            load_model_id=metadata.get("load_model_id"),
            loss=metadata.get("loss", 0.0),
            best_epoch=metadata.get("best_epoch"),
        )
        # Optional fields not in fillable need explicit assignment
        if "is_base_model" in metadata:
            checkpoint.is_base_model = metadata.get("is_base_model")
            checkpoint.save()

    dest_dir = os.path.join(CHECKPOINT_DIR, str(checkpoint.id))
    if os.path.exists(dest_dir):
        shutil.rmtree(dest_dir)
    os.makedirs(dest_dir, exist_ok=True)

    data_root = os.path.join(tmpdir, "data")
    if not os.path.isdir(data_root):
        data_root = tmpdir

    for root, _, files in os.walk(data_root):
        for fname in files:
            src_path = os.path.join(root, fname)
            if os.path.basename(src_path) == "metadata.json":
                continue
            rel_path = os.path.relpath(src_path, data_root)
            dest_path = os.path.join(dest_dir, rel_path)
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            shutil.copy2(src_path, dest_path)

    _cleanup_tempdir(tmpdir)
    return {
        "status": "success",
        "message": "Checkpoint imported",
        "checkpoint_id": checkpoint.id,
    }, 200
