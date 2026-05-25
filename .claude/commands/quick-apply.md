---
description: Sync source changes into the running /opt/easytrainer/project (no Docker rebuild)
---

Run [scripts/quick_apply.sh](../../scripts/quick_apply.sh) to copy local source
changes into the runtime project directory mounted by the `easy_collector_service`
container.

## When to invoke

- **After editing any code** under `backend/`, `frontend/`, `ros2/`, or `scripts/`.
  Without this, the running container keeps serving the old version and your changes
  appear to "do nothing."
- Not needed for changes under `docs/`, `home-next/`, `release/`, `training_server/`,
  or `modules/` (those have their own deployment paths — see [FOLDERS.md](../../FOLDERS.md)).

## What it does

```bash
bash /home/airlab/Easy-Collector/scripts/quick_apply.sh \
     /home/airlab/Easy-Collector \
     /opt/easytrainer/project
```

The script uses `rsync` (or `cp` fallback) to mirror `backend/`, `frontend/`,
`ros2_ws/src/`, the compose files, and helper scripts into the runtime project.
It does **not** rebuild Docker images. For deeper changes (Dockerfile,
requirements.txt, apt deps) use `scripts/rebuild_images.sh` instead.

## Verification

After running:

1. The script prints which files were copied. Spot-check one.
2. For backend changes that affect routes, hit the endpoint:
   `curl -s http://localhost:5000/api/<route>`
3. For frontend changes, the Quasar dev server picks up the new file via HMR;
   reload the browser if HMR misses it.
4. If the container is not running, the script will copy files anyway — the
   change will take effect on next `docker compose up -d service`.

## Notes

- This is the single canonical sync step; don't `docker cp` or edit files inside
  the container directly — they will be overwritten the next time anyone runs
  this command, and they won't survive a container restart.
- The `/opt/easytrainer/project` path is the runtime root used by the service
  entrypoint; it is bind-mounted, so changes are visible immediately.
