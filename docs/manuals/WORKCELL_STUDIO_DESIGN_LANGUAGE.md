# Workcell Studio design language

This document is the shared visual and interaction language of Workcell Studio (EMD) and Easy
Perception Deployment (EPD). Both are products of one company; each keeps its own identity and
layout:

- **EMD:** a light/dark engineering workspace.
- **EPD:** a dark perception console.

This language exists to remove inconsistency, not personality.

## Principles

1. **Precision over decoration.** Hierarchy comes from alignment, a 4 px grid, type weight and one
   accent. There are no gradients, glass or animation for their own sake.
2. **Nothing is ever clipped.** A label that does not fit changes form: it becomes an icon with a
   tooltip, wraps onto a new line, or scrolls. It is never cut to "Se" or "ave Layou".
3. **One primary action per page.** Secondary actions sit at their natural width in a wrapping
   action bar, not in a stack of full-width bars.
4. **State is never shown by font weight.** A bold selected tab or a bold checked button changes
   width and clips its text. State uses the accent colour, a soft background or an underline.
5. **Safety and readiness stay visible.** Badges such as FAKE HARDWARE, REAL LOCKED and BLOCKED are
   never hidden by layout modes.
6. **Remember the user's arrangement.** Window geometry and panel widths are restored, and
   **Reset Layout** always returns to the defaults.

## Tokens

The tokens are defined in `workcell_builder/include/workcell_builder_ui_utils.hpp` (`StudioTokens`) and
used as `@name` in stylesheets through `studioQss()`. EPD mirrors the dark values in
`windows/studio_design.py` (`TOKENS`).

| Role | Light | Dark |
|---|---|---|
| bg / surface / surfaceSunken | `#F5F7FA` / `#FFFFFF` / `#EEF2F6` | `#111720` / `#1A2230` / `#0E141C` |
| text / secondary / muted | `#172B3F` / `#5C6F82` / `#8A99AA` | `#E6EDF5` / `#A9B6C6` / `#7F8CA0` |
| border / borderStrong | `#DDE5EE` / `#C7D3DF` | `#2A3647` / `#3B4A5E` |
| accent / hover / soft | `#246FE5` / `#0B4CC1` / `#EAF1FF` | `#4C8DFF` / `#6AA2FF` / `#1B3159` |
| success / warning / danger | `#147A49` / `#B45309` / `#BC2B34` | `#3DD68C` / `#F5B14B` / `#FF7B7B` |
| radius S / M / L | 6 / 9 / 12 px | same |

EPD's primary action fill is `#246FE5`; white text on it has a contrast ratio of 4.6 : 1.

## Type, spacing and density

- **Type sizes.**
  - Base text: 13 px (EMD).
  - Captions: 11 px.
  - Page titles: 24 px / 700.
  - Section titles: 15 px / 650.
  - Nothing smaller than 10 px. EPD's former 9 px captions are raised by `scale_font_sizes`.
- **Spacing.** Multiples of 4 px.
  - Console pages: 24 px side and 16 px top margins, with 12 px between blocks.
  - Action bars: 8 px gaps.
  - Toolbar: 4 px gaps.
- **Density** (System → Appearance & Layout, or `appearance/density`; applies live):

  | Mode | Controls | Fields |
  |---|---|---|
  | Comfortable (default) | 30 px | 28 px |
  | Compact | 26 px | 24 px, with tighter padding |

- **Text size:** 100 / 112 / 125 % (`appearance/text_scale`; applies live in both apps).
- **Theme:** System / Light / Dark (`appearance/mode`; applies at the next start, because the
  tokens are fixed at launch).

## Components

| Component | Where | Behaviour |
|---|---|---|
| Studio icons | `studioIcon(name)` | Painted 16 px line icons (1.6 px stroke), rendered at 1× and 2×: select, move, rotate, add, fit, mate, undo, redo, save, preview, view, more, reset. No image assets. |
| Responsive toolbar | `installResponsiveToolbar()` | Three tiers chosen on resize: icon + label, then icon-only for secondary actions, then icon-only throughout except Save. The full name stays in the tooltip and accessible name. |
| Action bar | `makeActionBar()` | Wrapping flow layout. Buttons take their natural width and wrap rather than clip. |
| Console page | `polishConsolePage()` | Margins and rhythm, wrapped text, top alignment, action bars, and a vertical scroll body when the window is short. |
| Card | `QLabel#studioCard`, `QFrame#studioCard` | Surface, 1 px border, radius M, 14/10 px margins. Margins are real contents margins, because stylesheet padding breaks word-wrap height. |
| Readiness | EPD `ready` property | A quiet danger border when not ready. Legacy whole-button green/red paint is removed. |

## Layout and persistence

| Item | Behaviour |
|---|---|
| Window minimum size | 1024 × 640, so the 1280 × 720 constrained mode fits. |
| First launch | Maximized. |
| Later launches | Saved geometry, clamped to a present screen (`restoreWindowGeometryClamped`). |
| Scene Builder panel widths | Left 180–360 px, 3D view ≥ 440 px, inspector 476–560 px. User widths persist (only a laid-out splitter is saved). |
| Home table, narrow | Below 720 px, Robot and Modified hide; the selected-workcell panel shows them. |
| Reset Layout | System page, or Ctrl+Shift+0. Clears window and panel arrangement; appearance is kept. |
| EPD | Named splitters remembered (`SplitterMemory`). **View → Reset Layout** and **View → Text size** in the Deploy header. |

## Verification

Both GUIs carry a screenshot tour that builds the real application and audits every shot:

- **EMD:** `workcell_builder --ui-tour --workspace WS --scene ur5_2f_test --window-size WxH
  --smoke-output DIR --tag TAG`.
  - It visits every page, every Scene Builder tab and the New Cell wizard, then clicks Reset Layout.
  - It records clipped text, overflow from horizontally fixed scroll areas, vertically squeezed
    layouts and unreachable controls.
  - Without `--window-size` it verifies the restored geometry.
- **EPD:** `python3 scripts/ui_tour_capture.py --out DIR --size WxH --tag TAG` (run from `gui/`).

Run the tours with `QT_SCALE_FACTOR=1.25/1.5/2` for scaled checks, and with an isolated
`XDG_CONFIG_HOME` so your own settings are untouched.
