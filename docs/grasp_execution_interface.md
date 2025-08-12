# Grasp Execution Interface

The end effector execution interface provides helper methods for attaching
and detaching objects from the robot's end effector. It now supports an
optional execution context that allows callers to wait for hardware
operations to complete before proceeding and to insert delays before
issuing commands.

The `EndEffectorExecutionContext` exposes three tuning options:

* `pre_command_delay` – duration to wait before issuing a command. A zero
  duration disables the delay.
* `post_command_delay` – duration to wait after a command before returning.
  A zero duration (default) disables the delay.
* `wait_for_completion` – whether to respect the post-command delay. The
  default is `false` to match previous behaviour.

Passing a context instance to `grasp_object` or `release_object` applies the
configured timing to those operations.

## Usage

```c++
#include "emd/end_effector/ee_execution_interface.hpp"

emd::EndEffectorExecutioninterface ee;

// Operations without a context return immediately
ee.grasp_object(moveit_execution, "tool0", target_id);

emd::EndEffectorExecutionContext ctx;
ctx.pre_command_delay = std::chrono::milliseconds(250);
ctx.post_command_delay = std::chrono::milliseconds(500);
ctx.wait_for_completion = true;

// Attach an object, waiting briefly before and half a second after the command
ee.grasp_object(moveit_execution, "tool0", target_id, ctx);
// Release the object with the same delays
ee.release_object(moveit_execution, "tool0", target_id, ctx);
```

Setting `wait_for_completion` to `false` or omitting the context parameter
performs the operation without waiting after the command. The pre-command
delay, if specified, is always honoured.
