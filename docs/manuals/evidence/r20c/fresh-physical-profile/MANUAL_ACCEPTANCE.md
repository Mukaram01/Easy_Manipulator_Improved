# Fresh-cell production acceptance — pending user

Launch on HOME PC:

```bash
bash /tmp/r20c-production-acceptance-v2/launch.sh
```

1. Close older Studio windows. In Setup select Humble and `/home/user/workcell_ws`, then **Open Studio**.
2. **New Cell → Basics**: scene/package `r20c_home_ui_acceptance_v2_20260916`, Pick & Place. Keep the standard scenes output path.
3. **Robot**: UR5. **End Effector**: Robotiq / robotiq_85. **Environment → Use Recommended Layout**. Keep its reviewed workbench, physical bin, camera and source region unchanged.
4. **Task Intent**: Manual simulated source; source `pick_zone_main`, destination `target_bin_default`. Keep automatic grasp and placement. **Review → Create and Open**.
5. PASS: Product View reaches **ready**, identifies the new cell, and displays UR5, Robotiq, workbench, bin and camera. Capture a screenshot. If preparation fails, stop and send the session log directory; do not repair files.
6. Open the existing task editor. Confirm source `pick_zone_main`, physical target `target_bin_default`, region `default_drop_zone`. Set Object class `r20c_part`. Exercise Grasp policy AUTO, PREFERRED and EXACT (select an offered 2F strategy for a requested policy), then return grasp to AUTO. Exercise Placement policy PREFERRED then EXACT with local XYZ `0.01, 0, 0.01` and RPY `0, 0, 0`. **Validate task → Save task**. PASS: requested values/policies remain visible, no substitution.
7. **Home → open this same cell**. PASS: ready again; equipment/environment/source/destination and task values match step 6. Close Studio normally. The launch command saves `saved-state.json` and authored file copies in its printed session directory.
8. Run the same launch command again. Open the same cell through Home. PASS: identical task values and physical bindings; Web3D ready with the same identity.
9. Use the existing **Generate Scene Package**, then **Validate** actions. PASS: physical package generation/validation succeeds. This does not certify manipulation runtime parity: legacy execution recipes remain explicitly blocked for TaskIntent v2, which must use the shared resolver/preplanner.
10. In the task editor keep placement EXACT and set local XYZ `10, 0, 0`. **Validate task** must visibly block the outside-region request, preserve EXACT and the entered value, and never substitute AUTO or a valid pose. Capture a screenshot. Discard this invalid edit when navigating away; do not save it.
11. Reopen the saved cell, confirm step 6 values, then close Studio. Send both printed session directories, the two screenshots, and `PASS steps 1–11` or the exact failed step. Codex will compare recorded authored bindings and normalized TaskIntent hashes. No manual file editing or terminal repair is part of this acceptance.

R2.0c remains partial and PR #3170 remains draft until this acceptance succeeds.
