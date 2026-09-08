(() => {
  'use strict';

  // Qt performs a full page navigation when switching Product View scenes. The
  // browser therefore owns final destruction of the module graph, WebGL context,
  // DOM listeners and animation callbacks. This bridge gives the native shell a
  // deterministic, idempotent hand-off point before that navigation instead of
  // reporting a false "lifecycle_api_unavailable" warning.
  let disposed = false;

  function disposeScene(reason = 'qt_scene_navigation') {
    const normalizedReason = String(reason || 'qt_scene_navigation');
    if (disposed) {
      return {
        disposed: true,
        already_disposed: true,
        reason: normalizedReason,
      };
    }

    disposed = true;
    try {
      window.dispatchEvent(new CustomEvent('workcell:viewer-dispose', {
        detail: { reason: normalizedReason },
      }));
    } catch (_) {
      // A teardown acknowledgement must never block the following navigation.
    }

    return {
      disposed: true,
      already_disposed: false,
      reason: normalizedReason,
    };
  }

  Object.defineProperty(window, '__WORKCELL_VIEWER_LIFECYCLE__', {
    configurable: false,
    enumerable: false,
    writable: false,
    value: Object.freeze({
      apiVersion: '1.0.0',
      disposeScene,
    }),
  });
})();
