# Workcell Studio UI Layout

Workcell Studio now uses a **progressive disclosure** layout:

- Each page keeps one primary action and only a few secondary actions visible.
- Advanced/secondary actions are grouped under a **More Actions** dropdown.
- The Scene Builder is **canvas-first** with resizable/collapsible side panels.
- Mode indicators are shown in compact chips:
  - Design
  - Preview
  - Plan
  - Simulate
  - Hardware Guarded
- Safety gates remain enforced with compact messaging:
  - Runtime disabled by default
  - Fake hardware by default
  - Guarded execution
  - No uncontrolled robot motion

This keeps demo flows cleaner without removing any existing safety constraints or action coverage.
