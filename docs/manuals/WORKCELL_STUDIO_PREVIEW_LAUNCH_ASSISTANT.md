# Workcell Studio Preview Launch Assistant

Preview Launch Assistant provides a guided, fake-hardware-first preview workflow.

## Safety gates
- Fake hardware only (`use_fake_hardware:=true`).
- Real hardware and runtime execution flags are blocked.
- PREVIEW_ONLY placeholder scenes are not launch-enabled.
- BLOCKED acceptance scenes are not launch-enabled.

## When Run Preview is enabled
Only when scene acceptance/readiness is PASS/READY/WARNINGS and scene is not PREVIEW_ONLY.

## Copy commands
Use copy actions for build, source, launch, or all commands. All launch commands preserve `use_fake_hardware:=true`.

## Stop Preview
Stop Preview only stops the local preview process started by the assistant; it does not command controllers.

## Not supported yet
- Real robot hardware launch.
- Runtime execution enablement.
- Motion command dispatch.

## Troubleshooting
- Re-run acceptance and demo readiness.
- Verify `launch/demo.launch.py` exists in the generated scene package.
- Rebuild package and source workspace setup.
