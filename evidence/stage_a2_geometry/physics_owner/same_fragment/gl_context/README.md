# Live GL context diagnostic — BLOCKED

Baseline `000b99f729453f1c1aa45b3d6695161bb16121e5`, with existing manual
world-generator and runner-test modifications. These changes remove authored
lights and explicitly disable the grid; they were preserved. The initial patch
is retained in `manual_before.patch.gz`, and the prior manual attempt is retained
separately under `manual_before_attempt/`. No historical evidence was overwritten.

## Minimal implementation

The live capture witnesses GL state before and after the existing Ogre render
system's public `postExtraThreadsStarted()` lifecycle call. In Ogre-Next 2.2.5
GL3Plus this reacquires its existing cached current context using `setCurrent()`;
no context is constructed or cloned. Initialization and capture must remain on
the same checked thread. The installed public header and matching source were
inspected. Scene PreRender alone does not explicitly reacquire a GL context.

Witnesses include actual GL_VERSION, GL_RENDERER, integer major/minor, prior and
query glGetError, Linux TID and C++ thread ID, plus public EGL/GLX current-context
queries where available through process-global symbol lookup. Missing queries
are explicitly UNKNOWN. The diagnostic is flushed before the GL gate rejects.
The GL 4.5, typed uint clear, attachment, depth and complete-inventory gates remain
strict. Both new diagnostic headers are included in source provenance.

Reference implementation:
https://raw.githubusercontent.com/OGRECave/ogre-next/v2.2.5/RenderSystems/GL3Plus/src/OgreGL3PlusRenderSystem.cpp
(`postExtraThreadsStarted`). This is a supported existing-context reacquisition
method, not evidence that the resulting context meets GL 4.5 requirements.

## Sole fresh experiment

Output: `/tmp/stage_a2_gl_context_000b99f7_20261010`.
Runner uses `LIBGL_ALWAYS_SOFTWARE=1`, GDB diagnostic supervision and `timeout 60s`.
Exactly one attempt; child/supervisor exit 2, no signal or crash, no retry.
Owner ELF observed loaded; step-2 trace and owner/render ECM equivalence pass.
Complete SceneManager cube mapping passes with lights and grid absent.

Both before and after reacquisition:

- GL_VERSION: `4.3 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.4`.
- GL_RENDERER: `SVGA3D; build: RELEASE;  LLVM;`.
- GL_MAJOR_VERSION / GL_MINOR_VERSION: 4 / 3.
- prior glGetError: 0; query glGetError: 0.
- GLX query available, no current GLX context.
- EGL query unavailable through RTLD_DEFAULT; EGL context UNKNOWN, not absent.
- Linux TID 361073, C++ thread `129280807794240`, same render-owner thread.

Exact rejection: `GL4.5 typed clear API required: UNKNOWN_CONTEXT_API`.
The observed GL version independently falls below the unchanged required 4.5.
This is not evidence of a missing GL context: GL queries returned a coherent 4.3
version and renderer, but EGL context ownership is not independently witnessed.
There is no measured change from reacquisition. CPU classification retains the
unknown-query gate before declaring context authority.

stderr also records:
`libEGL warning: Not allowed to force software rendering when API explicitly selects a hardware device.`
Thus requesting software rendering did not produce the required llvmpipe backend
on this headless path. No RGB/ID buffers exist, no pixel counts are available,
and no MRT draw occurred. Render/physics timing and all contact/EPD authority
remain BLOCKED.

Executed capture ELF SHA256:
`722b14763a4161aa4e5bbdfaa5e65511415e5fd50ea67e161c69354d7dbcf847`.

Unchanged owner ELF SHA256:
`0f4a2dda8d7bfd67745b03493254a9d91ea66bb36dcb2d78440fbf5809ff072a`.

`attempt/preflight.json` pins exact source/world/executable/owner identities.
`attempt/result.json.gz` includes loaded-library hashes and full failure state;
`capture.json.gz` retains both context witnesses, mapping and inventory.
Process maps, full stdout/stderr, owner trace, execution claim, debugger script
and raw artifact SHA256 manifests are retained. Gzip compression is deterministic.

## Focused validation

- Opt-in live capture and native CPU targets rebuilt successfully.
- 98 Python tests passed: runner (including 29 manual runner tests), live fragment,
  fragment identity, visual/collision identity and owner registration.
- Five native tests passed: GL context classification, renderer diagnostics,
  lossless inventory comparison, owner bindings and owner identity inventory.
- New synthetic CPU cases reject absent/ambiguous/unqueryable contexts, GL 4.4,
  query/prior errors and missing strings; GL 4.5 clean state passes classification.
- Initial native regression failed against the unimplemented checker (retained).
- One initial CTest invocation raced the still-running build and could not find
  the new test executable. After the build finished, all three affected tests
  passed; both logs are retained. This was not a graphics attempt.
- Source/binary/owner hashes and unchanged manual runner tests verified after run.

No EPD, MoveIt, controllers, execution goals, ACM changes or simulator planning
pose substitution. Original envelopes and 0.1 mm threshold unchanged. No edits
to installed libraries or protected Stage-A1. PR remains draft.

## Exact next action

Use the proven native fixture's Ogre GLX/llvmpipe initialization path for this
disposable capture instead of the headless EGL hardware-device selection, with
an explicit current-context witness and the unchanged GL 4.5 gate. Validate in a
separately authorised bounded attempt; do not override the reported GL version
or weaken the typed-clear requirement.
