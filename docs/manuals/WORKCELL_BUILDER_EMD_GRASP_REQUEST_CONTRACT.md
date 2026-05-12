# Workcell Builder EMD Grasp Request Contract

Workcell Studio generates preview-only grasp strategy metadata and EMD grasp planner request artifacts.

- EPD is perception only.
- EMD owns grasp planning.
- Workcell Studio writes request artifacts only.
- This flow does not call the planner yet.
- This flow does not execute robot motion or gripper actions.
- Fake hardware remains the default.
- Point-cloud visualization from PR #1489 remains the debugging path when EMD planner is run later.
