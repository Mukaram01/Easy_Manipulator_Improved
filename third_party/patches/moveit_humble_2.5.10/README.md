# MoveIt Humble 2.5.10 lifecycle patches

These patches preserve the two MoveIt lifecycle fixes required by the
Workcell Studio Stage-A simulator commissioning environment.

They are stored as patches rather than vendoring MoveIt or preserving a
machine-specific overlay workspace.

## Upstream baselines

### moveit_ros_move_group

Version/tag:

    release/humble/moveit_ros_move_group/2.5.10-1

Base commit:

    66d37b40594e2b0ce8e8bd407122d20791d8c3b5

Patch:

    move_group_capability_loader_lifetime.patch

Purpose:

Keep the MoveGroup capability plugin loader alive until after the main ROS node
and executors are destroyed. Callback-group/entity control blocks may have
deleters implemented inside capability plugins, so unloading those plugins too
early can cause a teardown crash.

## moveit_ros_planning

Version/tag:

    release/humble/moveit_ros_planning/2.5.10-1

Base commit:

    c62753946ae3629a8cb745767844f7e69ca51489

Patch:

    trajectory_execution_manager_teardown.patch

Purpose:

During TrajectoryExecutionManager destruction:

- stop and join the continuous execution worker while controller callbacks still
  have a live executor;
- clear queued work;
- destroy active controller handles, callback handlers, controller manager and
  controller-manager node;
- unload the controller plugin loader only after those objects are gone.

This fixes teardown ownership/lifetime ordering without changing the public ABI.

## Application

Apply these patches to clean MoveIt Humble 2.5.10 package sources matching the
baselines above.

Example:

    git -C moveit_ros_move_group apply \
      /path/to/move_group_capability_loader_lifetime.patch

    git -C moveit_ros_planning apply \
      /path/to/trajectory_execution_manager_teardown.patch

Build them as an overlay.

Do not modify /opt/ros/humble in place.

These patches are supporting commissioning dependencies. They do not constitute
physical robot safety certification.
