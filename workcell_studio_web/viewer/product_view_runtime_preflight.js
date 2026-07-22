(() => {
  'use strict';

  const VERSION = 1;
  const LIGHT_BACKGROUND = Object.freeze({
    css: '#eef1f4',
    red: 0xee / 0xff,
    green: 0xf1 / 0xff,
    blue: 0xf4 / 0xff,
    alpha: 1,
  });
  const PATCH_FLAG = Symbol.for('workcell-studio.product-view-background-preflight.v1');

  function installClearColorOverride(prototype) {
    if (!prototype?.clearColor || prototype.clearColor[PATCH_FLAG]) return;
    const originalClearColor = prototype.clearColor;
    const forcedClearColor = function workcellProductViewClearColor(red, green, blue, alpha) {
      if (this?.canvas?.id === 'scene-canvas') {
        return originalClearColor.call(
          this,
          LIGHT_BACKGROUND.red,
          LIGHT_BACKGROUND.green,
          LIGHT_BACKGROUND.blue,
          LIGHT_BACKGROUND.alpha,
        );
      }
      return originalClearColor.call(this, red, green, blue, alpha);
    };
    forcedClearColor[PATCH_FLAG] = true;
    prototype.clearColor = forcedClearColor;
  }

  function installCanvasContextHook() {
    const prototype = window.HTMLCanvasElement?.prototype;
    if (!prototype?.getContext || prototype.getContext[PATCH_FLAG]) return;
    const originalGetContext = prototype.getContext;
    const hookedGetContext = function workcellProductViewGetContext(type, ...rest) {
      const context = originalGetContext.call(this, type, ...rest);
      if (context && (type === 'webgl' || type === 'webgl2' || type === 'experimental-webgl')) {
        installClearColorOverride(Object.getPrototypeOf(context));
      }
      return context;
    };
    hookedGetContext[PATCH_FLAG] = true;
    prototype.getContext = hookedGetContext;
  }

  installClearColorOverride(window.WebGLRenderingContext?.prototype);
  installClearColorOverride(window.WebGL2RenderingContext?.prototype);
  installCanvasContextHook();
  document.documentElement.style.setProperty('--workcell-product-view-background', LIGHT_BACKGROUND.css);

  window.__WORKCELL_PRODUCT_VIEW_RUNTIME_PREFLIGHT_V1__ = Object.freeze({
    enabled: true,
    version: VERSION,
    background: LIGHT_BACKGROUND.css,
    scope: 'scene-canvas-light-background',
  });
})();
