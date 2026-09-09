(() => {
  'use strict';

  // The bootstrap owns the stable host API. The bundle registers resource
  // cleanup without replacing this property. Remember early Qt teardown so a
  // late bundle cannot start a new renderer on a page being navigated away.
  if (window.__WORKCELL_VIEWER_LIFECYCLE__) return;
  let disposed = false;
  let disposeReason = '';
  let cleanup = null;

  function registerCleanup(implementation) {
    if (typeof implementation !== 'function') throw new TypeError('Viewer cleanup must be a function');
    if (cleanup && cleanup !== implementation) throw new Error('Viewer cleanup already registered');
    if (cleanup === implementation) return;
    cleanup = implementation;
    if (disposed) cleanup(disposeReason);
  }

  function disposeScene(reason = 'qt_scene_navigation') {
    const normalizedReason = String(reason || 'qt_scene_navigation');
    if (disposed) {
      return { disposed: true, already_disposed: true, reason: normalizedReason };
    }
    // Do not acknowledge successful teardown if resource cleanup throws.
    if (cleanup) cleanup(normalizedReason);
    disposed = true;
    disposeReason = normalizedReason;
    window.dispatchEvent(new CustomEvent('workcell:viewer-dispose', {
      detail: { reason: normalizedReason },
    }));
    return { disposed: true, already_disposed: false, reason: normalizedReason };
  }

  Object.defineProperty(window, '__WORKCELL_VIEWER_LIFECYCLE__', {
    configurable: false,
    enumerable: false,
    writable: false,
    value: Object.freeze({ apiVersion: '1.0.0', disposeScene, registerCleanup }),
  });
  window.addEventListener?.('pagehide', () => disposeScene('pagehide'), { once: true });
  window.addEventListener?.('beforeunload', () => disposeScene('beforeunload'), { once: true });
})();
