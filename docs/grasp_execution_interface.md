# Grasp Execution Interface

The end effector execution interface provides helper methods for attaching
and detaching objects from the robot's end effector.  It now supports an
optional execution context that allows callers to wait for hardware
operations to complete before proceeding.

## Usage

```c++
#include "emd/end_effector/ee_execution_interface.hpp"

emd::EndEffectorExecutioninterface ee;
emd::EndEffectorExecutionContext ctx;
ctx.wait_for_completion = true;
ctx.post_command_delay = std::chrono::milliseconds(500);

// Attach an object and wait half a second for the gripper to close
 ee.grasp_object(moveit_execution, "tool0", target_id, ctx);
```

Setting `wait_for_completion` to `false` skips the delay, matching the
previous behaviour.
