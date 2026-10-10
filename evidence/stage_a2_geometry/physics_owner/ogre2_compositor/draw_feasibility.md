# Draw-submission feasibility decision

Baseline `563bbc2714929cbb8ad46de1c544611add818770`; installed Ogre-Next development package `2.2.5+dfsg3-0ubuntu2`, ignition-rendering 6.6.4. **BLOCKED at the feasibility gate. Stop this GPU instrumentation route.** No draw observer was implemented, no build/test/fixture was run, and no installed library or protected Stage-A1 file was changed. This follows the task's conditional implementation/run gate; adding a partial listener would not deliver its complete inventory requirement.

## Specific supported API limitation

The installed public headers and upstream v2.2.5 source expose these boundaries:

| Candidate | Why it cannot supply the required complete draw witness |
| --- | --- |
| CompositorWorkspaceListener | Pass boundaries surround potentially many draws; not a callback after each final state binding and before each draw. Existing post-pass evidence stays unchanged. |
| RenderQueueListener | Queue-group boundaries, with skip/repeat controls; not individual draw dispatch. |
| RenderObjectListener | Legacy SceneManager notification sites do not wrap the fast RenderQueue command dispatch or its immediate renderSingleObject path. A renderable notification is also not an exhaustive GL draw inventory. |
| HlmsListener | Installed OgreHlms.h explicitly documents that listeners are not called per object. Hlms pre/postCommandBufferExecution are overridable implementation methods around a whole command buffer, not observer subscriptions around its individual commands. Immediate quads bypass that command-buffer boundary. |
| RenderSystem::Listener | Custom named events; the inspected GL3Plus implementation supplies no per-draw event. Virtual draw methods are renderer implementation points, not an observer attached to the already loaded renderer. |
| OpenGL KHR_debug | Callback on generated debug messages; no guarantee of one message per draw or its complete inputs. Synchronous debug output does not add that guarantee. |

Pinned control flow: RenderQueue.cpp lines 460–468 calls HLMS hooks then CommandBuffer::execute; OgreCommandBuffer.cpp lines 95–109 dispatches its command table then clears the buffer. CompositorPassQuad.cpp lines 282–284 performs delayed descriptor actions and calls SceneManager::_renderSingleObject; SceneManager.cpp lines 4420–4424 delegates to RenderQueue::renderSingleObject, whose lines 890–898 bind state and call rs->_render directly. These paths lack a shared supported read-only draw observer. Public command lookup methods are not an execution callback or proof of complete inventory. Replacing the internal dispatch table, interposing GL entry points, or replacing the loaded renderer is outside this gate's supported read-only callback route; none was attempted.

The conclusion is scoped to these reviewed facilities and the required existing runtime, not a claim that every possible tracing tool or future Ogre API is impossible. Upstream control-flow inspection is not direct proof of the distribution binary's execution. `draw_feasibility.json` retains exact upstream source and installed header/library hashes separately. Upstream tag v2.2.5 resolves to `0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4`; source URLs use that immutable commit below.

- [RenderQueue](https://github.com/OGRECave/ogre-next/blob/0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4/OgreMain/src/OgreRenderQueue.cpp#L460)
- [Command dispatch](https://github.com/OGRECave/ogre-next/blob/0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4/OgreMain/src/CommandBuffer/OgreCommandBuffer.cpp#L95)
- [Immediate quad](https://github.com/OGRECave/ogre-next/blob/0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4/OgreMain/src/Compositor/Pass/PassQuad/OgreCompositorPassQuad.cpp#L282)
- [HLMS extension scope](https://github.com/OGRECave/ogre-next/blob/0e0c47ed70091e7bdead5fb1ca01e1cae5857ef4/OgreMain/include/OgreHlms.h#L772)
- [Khronos debug callback contract](https://github.com/KhronosGroup/OpenGL-Refpages/blob/main/gl4/glDebugMessageCallback.xml)

## Authority remains separated

A: complete CPU-submitted draw state **UNKNOWN**. B: earlier post-pass GL queries observe state only at those boundaries; exhaustive driver-accepted draw state **UNKNOWN**. C: GPU-executed operations **UNKNOWN**. D: total pixel/depth error **UNKNOWN**; earlier matrix-only discrepancy ≤0.000035238669374079335 pixels remains only that claim. E: physical collision-to-EPD identity association **BLOCKED**. No new GPU inputs were witnessed. Existing mask remains **0 qualified / 262144 excluded**; all original safety gates stay closed.

The requested draw-specific negative tests cannot validate an absent draw implementation and were not fabricated. Existing 55 Python/CPU CTest results belong to the preceding commit, not this feasibility decision. At inspection, all four current-head PR checks (Humble, Jazzy, two security checks) passed on baseline 563bbc27. Any evidence-only follow-up commit needs its own CI status. No Gazebo/EPD or MoveIt was started.

## One alternative architecture; not implemented here

Pursue **one RGB plus integer entity-ID MRT acquisition sharing the same visible fragments**, using the existing Gazebo visual identities with an explicit, inventory-checked visual → link → collision mapping. Feed the exact RGB bytes to genuine EPD; bind returned masks through image/session/exposure hashes and verified preprocessing coordinates to the sibling ID attachment. Reject mixed/unknown IDs, unsupported transparency/MSAA, incomplete mappings, occlusions and ambiguous masks. Never use majority labels or nominal projection overlap as identity proof. Require the identity attachment to share the RGB fragment visibility/coverage contract; qualify that contract and ID encoding before granting association. This is a proposed architecture, not a claim that the current renderer already exposes it.

Carry only identity, capture/physics-step alignment witnesses and association evidence into the simulation-only contact gate. Keep EPD poses and original conservative envelopes unchanged; acquire and qualify DART depth evidence independently at the corresponding physics step, including bounded temporal motion or reject. This removes the three independently rendered image-registration dependency rather than presuming a new numerical tolerance. It still requires renderer/identity/timing qualification and grants no contact permission by itself.

**Exact next action:** establish feasibility of emitting a Gazebo entity-ID attachment alongside the *same RGB fragment outputs*, with an authoritative visual-to-collision inventory and an unchanged genuine EPD input image. Do not extend the current post-pass GPU instrumentation further.
