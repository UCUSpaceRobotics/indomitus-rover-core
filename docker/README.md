
## Docker

### !!!ATTENTION!!!⚠️

> ⚠️ Each user must set up `docker-compose` individually — drivers and dependencies vary by platform.

> Copy `docker/docker-compose.dev.example.yml` to the project root and rename it to `docker-compose.yml`.

### Dependencies
```bash
curl -fsSL https://get.docker.com | sh  # installs docker + compose plugin
sudo usermod -aG docker $USER           # run docker without sudo (re-login required)
```

### Quick Reference

| Action | Command |
|--------|---------|
| Build | `docker compose build` |
| Force rebuild | `docker compose build --no-cache` |
| Create + start | `docker compose up -d` |
| Enter container | `docker compose exec indomitus_rover_dev bash` |
| Stop (keep container) | `docker compose stop` |
| Start stopped container | `docker compose start` |
| Stop + delete | `docker compose down` |

> 💡 You can enter the same container from multiple terminals simultaneously.

### Display Access (for GUI / RViz)

Allow Docker to use your screen before entering the container:

```bash
xhost +local:docker
```

### Troubleshooting

- Commands not found? Try `docker-compose` (with `-`) or prepend `sudo`
- Container already exists? `docker compose up -d` will just start it, not recreate
