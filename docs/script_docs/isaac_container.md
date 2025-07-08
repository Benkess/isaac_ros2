# Isaac Sim + ROS 2 Container Launcher

This document explains how to use the `isaac_container.sh` script to launch the Apptainer-based Isaac Sim + ROS 2 container in different modes: **interactive shell**, **GUI**, and **headless**.

> **Note:** on the UVA CS server Apptainer is installed as a module. Use: 
> ```bash
> source /etc/profile.d/modules.sh
> module load apptainer
> ```

> Notes:
> - All commands assume the system admin setup this module according to **[Host Setup Guide](/docs/host_setup.md)**.
> - All host-side dirs (/var/cache/isaac/..., /projects, etc.) should exist before you run these commands.
> - You must have the correct directory permissions granted to the isaac group.
> - Your container already has EULA acceptance built-in via environment variables.
> - The same container can run headless, GUI, or WebRTC sessions with ROS 2 integration.
> - Isaac Sim may take a while to load on startup.


---

## 🚀 Usage

The script is located at scripts/isaac_container.sh in this module. All commands must be exicuted from that directory.

### 🔧 Launch Modes

#### 1. **Interactive Shell**

```bash
./isaac_container.sh shell
```

Drops into the container shell environment with ROS 2 and Isaac Sim runtime.

#### 2. **GUI Mode** (Local only)

```bash
./isaac_container.sh gui
```

Launches Isaac Sim with full GUI support.

> ✅ Requires access to an X11 display (e.g. `xhost +local:` may be needed).

#### 3. **Headless Mode** (e.g. for servers or scripting)

```bash
./isaac_container.sh headless
```

Runs Isaac Sim without GUI, useful for remote simulation, WebRTC, or automation.

---

## 🔄 Environment Variables

The container sets the following by default:

* `ACCEPT_EULA=Y`
* `PRIVACY_CONSENT=Y`

These are required for accessing Omniverse cloud assets.

---

## 📁 Cache and Data Mapping

The script binds the following paths:

| Host Path                       | Container Path                 | Purpose                     |
| ------------------------------- | ------------------------------ | --------------------------- |
| `/var/cache/isaac/kit`          | `/isaac-sim/kit/cache`         | Isaac Kit cache             |
| `/var/cache/isaac/ov`           | `$HOME/.cache/ov`              | Omniverse asset cache       |
| `/var/cache/isaac/pip`          | `$HOME/.cache/pip`             | pip cache                   |
| `/var/cache/isaac/glcache`      | `$HOME/.cache/nvidia/GLCache`  | GPU shader cache            |
| `/var/cache/isaac/computecache` | `$HOME/.nv/ComputeCache`       | GPU compute cache           |
| `/var/cache/isaac/logs`         | `$HOME/.nvidia-omniverse/logs` | Isaac Sim logs              |
| `/var/cache/isaac/data`         | `$HOME/.local/share/ov/data`   | Omniverse data              |
| `/projects`                     | `$HOME/Documents`              | User project directory      |
| `/persistent/isaac/asset_root`  | `/persistent/isaac/asset_root` | Local persistent asset root |

---

## ✅ Tips

* Use `--cleanenv` if you want to ignore host environment variables.
* To allow GUI on remote X11 connections, run: `xhost +local:` on the host.
* Logs and assets will persist across container launches.

---

## 📬 Contact

For help or customizations, contact your container admin or simulation infrastructure maintainer.
