# Host Setup
This guide covers container setup for the host. It also includes troubleshooting info.

## Environment Variables

### Key Environment Variables

The container sets these environment variables:

```bash
export ACCEPT_EULA=Y                    # Isaac Sim EULA acceptance
export PRIVACY_CONSENT=Y               # Privacy consent
export ROS_DISTRO=humble               # ROS 2 distribution
export ISAAC_NUCLEUS_ROOT=/persistent/isaac/asset_root
export ROS_SETUP=/opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
```

### Custom Environment Variables

You can override or add environment variables:

```bash
apptainer exec --nv --contain \
  --env MY_CUSTOM_VAR=value \
  --env ROS_DOMAIN_ID=42 \
  /containers/isaac_ros2_humble.sif
```

## Host Directory Setup

### Required Directories

Create these directories on your host system:

```bash
# Container storage
sudo mkdir -p /containers
sudo chmod 755 /containers

# Create all the cache directories you'll bind into the container
sudo mkdir -p /var/cache/isaac/{kit,ov,pip,glcache,computecache,logs,data}

# Give the isaac group ownership and group-writable perms
sudo chown -R root:isaac /var/cache/isaac
sudo chmod -R 2775 /var/cache/isaac

# Persistent asset root (optional but recommended)
sudo mkdir -p /persistent/isaac/asset_root
sudo chmod 755 /persistent/isaac/asset_root

# Projects directory (optional)
sudo mkdir -p /projects
sudo chmod 755 /projects
```

### Directory Permissions

For multi-user environments:

```bash
# Create isaac group
sudo groupadd isaac

# Add users to isaac group
sudo usermod -a -G isaac $USER

# Set group ownership
sudo chown -R root:isaac /var/cache/isaac /persistent/isaac
sudo chmod -R 2775 /var/cache/isaac /persistent/isaac
```


## Building the Container

```bash
# Build with default Isaac Sim 4.5.0
apptainer build /containers/isaac_ros2_humble.sif IsaacROS2.def

# Build with a specific Isaac Sim version
apptainer build --build-arg ISAAC_VERSION=5.0.0 /containers/isaac_ros2_humble.sif IsaacROS2.def
```

## Troubleshooting

### Common Issues

#### 1. No ROS 2 Topics Visible

**Problem:** `ros2 topic list` shows no topics

**Solutions:**
- Ensure Isaac Sim launched with ROS 2 bridge extension
- Check that `FASTRTPS_DEFAULT_PROFILES_FILE` is set
- Verify ROS 2 environment is sourced

```bash
# Inside container
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/exts/isaacsim.ros2.bridge/humble/config/default_profiles.xml
```

#### 2. Permission Errors

**Problem:** Cannot write to cache directories

**Solutions:**
- Check directory permissions on host
- Ensure bind mounts use `:rw` flag
- Verify user has access to mounted directories

```bash
# Fix permissions
sudo chmod -R 755 /var/cache/isaac
sudo chown -R $USER /var/cache/isaac
```

#### 3. Graphics/Display Issues

**Problem:** Isaac Sim won't start in GUI mode

**Solutions:**
- Ensure `--nv` flag is used
- Check X11 forwarding for remote systems
- Verify NVIDIA drivers are compatible

```bash
# Test NVIDIA access
nvidia-smi

# For remote X11
ssh -X user@remote_host
export DISPLAY=:0
```

#### 4. Slow Startup Times

**Problem:** Isaac Sim takes a long time to start

**Solutions:**
- Always bind mount cache directories
- Use persistent asset root
- Pre-populate cache directories

```bash
# Verify cache mounts
ls -la /root/.cache/kit    # Should show cached files
ls -la /root/.cache/ov     # Should show Omniverse cache
```

#### 5. Build Failures

**Problem:** Container build fails with package conflicts

**Solutions:**
- Clean previous build attempts
- Check for third-party repositories in base image
- Use ROS 2 base instead of desktop packages

```bash
# Clean build
rm -f /containers/isaac_ros2_humble.sif

# Rebuild
apptainer build /containers/isaac_ros2_humble.sif IsaacROS2.def
```

### Performance Optimization

#### Cache Strategy

1. **Always use cache directories:**
   ```bash
   --bind /var/cache/isaac/kit:/root/.cache/kit:rw \
   --bind /var/cache/isaac/ov:/root/.cache/ov:rw
   ```

2. **Pre-populate caches:**
   ```bash
   # First run will populate caches
   # Subsequent runs will be much faster
   ```

3. **Use SSD storage for caches:**
   ```bash
   # Place cache directories on fast storage
   sudo mkdir -p /fast/cache/isaac/kit
   sudo ln -s /fast/cache/isaac/kit /var/cache/isaac/kit
   ```

#### Resource Allocation

1. **GPU Memory:**
   - Monitor GPU memory usage with `nvidia-smi`
   - Adjust Isaac Sim settings for available VRAM

2. **CPU Resources:**
   - Use `--cpus` flag to limit CPU usage if needed
   - Monitor with `htop` or `top`

3. **Disk I/O:**
   - Use fast storage for cache directories
   - Monitor I/O with `iotop`

### Networking

#### ROS 2 Networking

1. **Domain ID:**
   ```bash
   --env ROS_DOMAIN_ID=42
   ```

2. **Network interfaces:**
   ```bash
   --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. **Multi-machine setup:**
   ```bash
   --env ROS_LOCALHOST_ONLY=0
   ```

#### Isaac Sim Networking

1. **WebRTC streaming:**
   ```bash
   --env WEBRTC_ENABLE=1
   -p 9090:9090
   ```

2. **Nucleus server:**
   ```bash
   --env ISAAC_NUCLEUS_ROOT=/persistent/isaac/asset_root
   ```

### Debugging

#### Container Debugging

1. **Interactive shell:**
   ```bash
   apptainer shell --nv --contain /containers/isaac_ros2_humble.sif
   ```

2. **Check environment:**
   ```bash
   # Inside container
   env | grep ROS
   env | grep ISAAC
   which python3
   which ros2
   ```

3. **Verify installations:**
   ```bash
   # Check ROS 2
   ros2 --help
   
   # Check Isaac Sim
   ls -la /isaac-sim/
   
   # Check extensions
   ls -la /exts/isaacsim.ros2.bridge/
   ```

#### Log Analysis

1. **Isaac Sim logs:**
   ```bash
   # Check Isaac Sim output
   tail -f ~/.local/share/ov/pkg/isaac-sim-*/logs/
   ```

2. **ROS 2 logs:**
   ```bash
   # Enable ROS 2 logging
   export RCUTILS_LOGGING_SEVERITY=DEBUG
   ```

3. **Container logs:**
   ```bash
   # Run with verbose output
   apptainer exec --debug --nv --contain /containers/isaac_ros2_humble.sif
   ```

## Advanced Usage

### Custom Build Arguments

Build with different Isaac Sim versions:

```bash
# Build with Isaac Sim 4.6.0
apptainer build --build-arg ISAAC_VERSION=4.6.0 \
  /containers/isaac_ros2_humble_4.6.0.sif IsaacROS2.def
```

### Multi-Container Workflows

Run multiple instances:

```bash
# Terminal 1: Isaac Sim
apptainer exec --nv --contain \
  --env ROS_DOMAIN_ID=1 \
  /containers/isaac_ros2_humble.sif --headless

# Terminal 2: ROS 2 tools
apptainer exec --nv --contain \
  --env ROS_DOMAIN_ID=1 \
  /containers/isaac_ros2_humble.sif \
  bash -c "source /opt/ros/humble/setup.bash && rviz2"
```

### CI/CD Integration

Example GitHub Actions workflow:

```yaml
name: Isaac Sim Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: self-hosted  # Requires GPU
    steps:
      - uses: actions/checkout@v3
      - name: Run Isaac Sim tests
        run: |
          apptainer exec --nv --contain \
            --bind $PWD:/workspace:rw \
            /containers/isaac_ros2_humble.sif \
            bash -c "cd /workspace && ./run_tests.sh"
```