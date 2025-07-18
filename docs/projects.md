# Projects & Overlays
This describes the recommended approach for creating Isaac Sim or ROS2 Projects using the included containers.

> **Note:**
> - See **[ROS2 Projects](/docs/ROS2/ros2_projects.md)** for more detail on ROS2 projects.

## Isaac Sim & ROS2 Projects:
It is recommended that your project directory is binded to the container. This allows you to read, write and exicute from the directory while in the container.

It is also recommended to backup your projects with git.

If you need to modify a container, it is recoomended to use a writable filesystem image overlay.

Optionally, you can store you project in /projects

## Recomended Project Layout:
For Isaac Sim and ROS2 projects it is recommended to have the following files and directories located in /projects. Note that these are optional and may not all be needed for your project.

- /projects
    - /ros2_ws
    - /isaac-sim/documents
    - ros2_overlay.img
    - isaac_overlay.img

## Binding a Project Directory:

To bind a project directory use:
```bash
  --bind    /path/to/<your-project-directory>:/path/to/<your-project-directory>
```

Here are some examples:
```bash
apptainer shell --nv --no-mount /l \
  --bind /projects:/projects:rw \
  /containers/isaac_ros2_humble.sif
```

```bash
apptainer shell --nv --no-mount /l \
  --bind /projects/<your-project>/isaac-sim/documents:$HOME/Documents:rw \
  /containers/isaac-sim.sif
```

```bash
apptainer exec --nv --no-mount /l \
  --overlay /projects/<your-project>/ros2_overlay.img \
  --bind    /projects/<your-project>/ros_ws:/ros_ws:rw \
  /containers/ros2_humble.sif \
```

## Filesystem image overlay
You can create a sparse overlay with fakeroot and a size of 1GB using the apptainer overlay create command with the --sparse, --fakeroot, and --size flags.
Here's the command:
Bash
apptainer overlay create --sparse --fakeroot --size 1024 overlay.img


--sparse: This flag ensures that the overlay image only takes up disk space as data is written to it, saving you space.
--fakeroot: This flag is used when you intend to modify the container with the overlay as a non-root user. It makes the overlay image writable in fakeroot mode.
--size 1024: This specifies the maximum size of the overlay in megabytes. In this case, 1024 MB is equivalent to 1 GB.
overlay.img: This is the name of the overlay image file that will be created. You can choose any name you like.
After creating the overlay, you can then use it with your Apptainer container, for example:
Bash
apptainer shell --fakeroot --overlay overlay.img ubuntu.sif


This will give you a shell inside the ubuntu.sif container, where you can install software or make changes, and these changes will be persisted in your overlay.img file.
