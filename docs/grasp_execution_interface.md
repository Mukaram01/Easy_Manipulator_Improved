# Grasp Execution Interface

The end effector execution interface provides helper methods for attaching
and detaching objects from the robot's end effector. It now supports an
optional execution context that allows callers to wait for hardware
operations to complete before proceeding.

The `EndEffectorExecutionContext` exposes two tuning options:

* `post_command_delay` – duration to wait after a command before returning.
  A zero duration (default) disables the delay.
* `wait_for_completion` – whether to respect the delay. The default is
  `false` to match previous behaviour.

Passing a context instance to `grasp_object` or `release_object` applies the
configured timing to those operations.

## Usage

```c++
#include "emd/end_effector/ee_execution_interface.hpp"

emd::EndEffectorExecutioninterface ee;

// Operations without a context return immediately
ee.grasp_object(moveit_execution, "tool0", target_id);

emd::EndEffectorExecutionContext ctx;
ctx.wait_for_completion = true;
ctx.post_command_delay = std::chrono::milliseconds(500);

// Attach an object and wait half a second for the gripper to close
ee.grasp_object(moveit_execution, "tool0", target_id, ctx);
// Release the object with the same delay
ee.release_object(moveit_execution, "tool0", target_id, ctx);
```

Setting `wait_for_completion` to `false` or omitting the context parameter
performs the operation without waiting.
