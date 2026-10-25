# Continuous controller collision certificate

Scope: Stage A1 shared auditor; recovery and hardware stay blocked until qualification.

1. Reproduce unchanged 1 ms quintic false acceptance; pin installed JTC identity.
2. Add the saved counterexample as an adapter regression (red before implementation).
3. Add bounded adaptive interval certification ahead of existing chronological policy validation.
   Bound the complete linear/cubic/quintic polynomial, including interior extrema and mimic joints.
   Derive per-body spatial bounds from all ancestor joints, offsets and collision geometry.
   Query complete robot/world and self distances using existing FCL scenes and ACM.
   Self relative bounds include both bodies. Unknown geometry/motion/limits reject.
   Conditional contacts require a separate analytic certificate; never treat an allowed
   sampled contact or omitted distance as continuous proof. Unsupported contacts reject.
4. Exercise counterexample, clear controls, extrema, limits, spatial bounds and real Stage A data.
5. Run focused existing C++ and Python regressions, independent review, diff check.
6. Commit/push only after all auditor gates pass; then consider detached withdrawal qualification.

Evidence directory: /home/ubuntu/workcell_ws/stage-a1-continuous-jtc-20260924.
