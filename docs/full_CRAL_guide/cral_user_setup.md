# CRAL User Setup

This describes the user setup process for this package. **Normal users should start here.** All commands should be run in a terminal window.

---

## Accessing the Lab Computer on the UVA CS Server

The CRAL lab computer is hosted on the UVA CS Server. First, log in to the server.

### Local Use

1. Turn on the monitor.
2. Add a new user if needed.
3. Sign in with your UVA CS Server credentials.

### Remote Use

1. SSH into the UVA CS Server:

   ```bash
   ssh your_computing_ID@portal.cs.virginia.edu
   ```

2. Find the CRAL computer’s address if needed:

   ```bash
   host cral
   ```

3. SSH into CRAL:

   ```bash
   ssh your_computing_ID@cral.cs.virginia.edu
   ```

---

## Apptainer Module
On the UVA CS Server Apptainer is a module. It is needed to use Isaac Sim. Follow these steps to load Apptainer:
1. Source module setup if needed:

   ```bash
   source /etc/profile.d/modules.sh
   ```

2. Load the Apptainer module:

   ```bash
   module load apptainer
   ```

## Isaac Container Setup
Users must do the following to use the container:
1. Get permission:
    
    Currently only the group isaac has the needed permissions. Ask someone with sudo access to add you or add yourself. 
    ```bash
    sudo usermod -a -G isaac $USER
    ```

## Next Steps

For usage guidance, check out:

* **[Usage Examples](/docs/usage.md)** — GUI mode, WebRTC, overlays, troubleshooting
* **[Humble WS](/docs/Isaac_Sim_humble_ws.md)** — Isaac Sim ROS2 tutorials

---

## Coming Soon...

This page is incomplete. Please see the **[README](/README.md)** for full details.

---
