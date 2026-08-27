# Fix: `unknown or invalid runtime name: nvidia`

**Platform:** Jetson Orin NX (reComputer J4012), JetPack 6 / L4T r36.4
**Symptom:** `docker compose up` fails with:
```
Error response from daemon: unknown or invalid runtime name: nvidia
```

## Root causes (check in this order)

There are three independent things that can cause this. On a fresh flash, all three can be broken at once.

1. `nvidia-container-toolkit` not installed
2. `containerd-snapshotter` feature enabled in Docker — silently drops custom runtimes
3. Docker CLI pointed at the **rootless** context instead of the system daemon

---

## Step 1 — Diagnose

```bash
# What runtimes does the CLI actually see?
docker info | grep -i -A2 -E 'Runtime|containerd'

# Which daemon is the CLI talking to?
docker context ls
```

Look at the output:

| Symptom | Cause |
|---|---|
| `Runtimes: runc` only, no `nvidia` | daemon.json missing/wrong, or containerd-snapshotter is on |
| `driver-type: io.containerd.snapshotter.v1` present | containerd-snapshotter is on → strips custom runtimes |
| `docker context ls` shows `rootless *` active | CLI is on the wrong daemon entirely |
| `failed to connect ... docker.sock: no such file or directory` | socket didn't get created (see Step 5) |

---

## Step 2 — Confirm nvidia-container-toolkit is installed

```bash
dpkg -l | grep nvidia-container
```

Should list `nvidia-container-toolkit`, `nvidia-container-toolkit-base`, `libnvidia-container1`, `libnvidia-container-tools`. If missing:

```bash
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

---

## Step 3 — Fix `/etc/docker/daemon.json`

Set it to exactly this (generate the runtime block automatically, then add the snapshotter fix by hand):

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

Then edit `/etc/docker/daemon.json` so it includes **both** the runtime block and `containerd-snapshotter: false`:

```json
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "features": {
        "containerd-snapshotter": false
    }
}
```

> **Why `containerd-snapshotter: false` matters:** if this feature is on (it can be default on some Docker builds), Docker only exposes `io.containerd.runc.v2` and `runc`. It ignores the `runtimes` block from daemon.json entirely — no error, no warning in logs. This is the most common silent cause of this whole issue.

---

## Step 4 — Make sure the CLI is on the system daemon, not rootless

Jetson/JetPack images can ship with a rootless Docker daemon running per-user (`dockerd-rootless.sh`) alongside the normal system one. If the CLI context is set to `rootless`, it talks to a completely separate daemon with its own separate config — your `daemon.json` fix will have zero effect.

```bash
docker context ls
```

If `rootless *` is active:

```bash
docker context use default
echo $DOCKER_HOST     # should be empty — if set, it overrides context; remove it from ~/.bashrc
```

Also disable the rootless daemon so it can't cause this again:

```bash
systemctl --user disable --now docker
```

> Note: `rootless` mode also can't do `privileged: true`, `network_mode: host`, or raw `/dev` access properly — all of which this project's compose file needs. So `default` context is required regardless of the runtime issue.

---

## Step 5 — Restart and verify

```bash
sudo systemctl restart docker.socket
sudo systemctl restart docker.service

# socket file must exist
ls -la /var/run/docker.sock

# confirm nvidia is now registered
docker info | grep -i -A2 -E 'Runtime|containerd'
```

Expected good output:
```
Runtimes: io.containerd.runc.v2 nvidia runc
Default Runtime: nvidia
```
(no `driver-type: io.containerd.snapshotter.v1` line)

If `/var/run/docker.sock` is missing after `systemctl status docker` shows "active": restart `docker.socket` **before** `docker.service` — socket activation sometimes doesn't create the file if the socket unit wasn't (re)triggered.

---

## Step 6 — Bring the container up

```bash
docker rm -f rover_prod   # only if a stale container exists from a failed attempt
docker compose up -d --wait
```

---

## Quick reference: full known-good `/etc/docker/daemon.json`

```json
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "features": {
        "containerd-snapshotter": false
    }
}
```

## Checklist after a fresh Jetson flash

- [ ] `nvidia-container-toolkit` installed
- [ ] `/etc/docker/daemon.json` matches the block above
- [ ] `docker context ls` → `default` is active (not `rootless`)
- [ ] Rootless docker service disabled: `systemctl --user disable --now docker`
- [ ] `docker info` shows `nvidia` in Runtimes and as Default Runtime