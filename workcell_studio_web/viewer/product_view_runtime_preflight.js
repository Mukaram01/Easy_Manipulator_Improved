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
  const STAGED_ASSET_PREFIX = 'build/workcell_studio_web_scene/assets';
  const SAFE_SCENE_ID = /^[A-Za-z0-9][A-Za-z0-9_-]*$/;
  const SAFE_PACKAGE = /^[A-Za-z][A-Za-z0-9_]*$/;
  const PATCH_FLAG = Symbol.for('workcell-studio.product-view-runtime-preflight.v1');

  function activeSceneId() {
    const scenePath = new URLSearchParams(window.location.search).get('scene') || '';
    const normalized = scenePath.replace(/^\/+/, '');
    const prefix = 'build/workcell_studio_web_scene/';
    const suffix = '.web_scene.json';
    if (!normalized.startsWith(prefix) || !normalized.endsWith(suffix)) return '';
    const filename = normalized.slice(prefix.length).split('/').pop() || '';
    const sceneId = filename.slice(0, -suffix.length);
    return SAFE_SCENE_ID.test(sceneId) ? sceneId : '';
  }

  function isPackageRootCandidate(packageName, rest) {
    if (!SAFE_PACKAGE.test(packageName) || !rest || rest.includes('..')) return false;
    if (!rest.startsWith('meshes/')) return false;
    return packageName === 'workcell_builder' || packageName.endsWith('_description');
  }

  function rewritePackageRootUrl(value) {
    const original = String(value || '');
    if (!original) return original;

    let url;
    try {
      url = new URL(original, window.location.href);
    } catch (_) {
      return original;
    }
    if (url.origin !== window.location.origin) return original;

    const match = url.pathname.match(/^\/([A-Za-z][A-Za-z0-9_]*)\/(.+)$/);
    if (!match) return original;
    const [, packageName, rest] = match;
    if (!isPackageRootCandidate(packageName, rest)) return original;

    const sceneId = activeSceneId();
    if (!sceneId) return original;
    url.pathname = `/${STAGED_ASSET_PREFIX}/${sceneId}/${packageName}/${rest}`;
    return url.href;
  }

  function installFetchRewrite() {
    if (typeof window.fetch !== 'function' || window.fetch[PATCH_FLAG]) return;
    const RequestConstructor = window.Request;
    const isRequest = input => typeof RequestConstructor === 'function' && input instanceof RequestConstructor;
    const originalFetch = window.fetch.bind(window);
    const rewrittenFetch = function workcellProductViewFetch(input, init) {
      const originalUrl = isRequest(input) ? input.url : input;
      const rewrittenUrl = rewritePackageRootUrl(originalUrl);
      if (rewrittenUrl === String(originalUrl || '')) return originalFetch(input, init);
      if (isRequest(input)) return originalFetch(new RequestConstructor(rewrittenUrl, input), init);
      return originalFetch(rewrittenUrl, init);
    };
    rewrittenFetch[PATCH_FLAG] = true;
    rewrittenFetch.originalFetch = originalFetch;
    window.fetch = rewrittenFetch;
  }

  function installXhrRewrite() {
    const prototype = window.XMLHttpRequest?.prototype;
    if (!prototype?.open || prototype.open[PATCH_FLAG]) return;
    const originalOpen = prototype.open;
    const rewrittenOpen = function workcellProductViewXhrOpen(method, url, ...rest) {
      return originalOpen.call(this, method, rewritePackageRootUrl(url), ...rest);
    };
    rewrittenOpen[PATCH_FLAG] = true;
    prototype.open = rewrittenOpen;
  }

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

  installFetchRewrite();
  installXhrRewrite();
  installClearColorOverride(window.WebGLRenderingContext?.prototype);
  installClearColorOverride(window.WebGL2RenderingContext?.prototype);
  installCanvasContextHook();
  document.documentElement.style.setProperty('--workcell-product-view-background', LIGHT_BACKGROUND.css);

  window.__WORKCELL_PRODUCT_VIEW_RUNTIME_PREFLIGHT_V1__ = Object.freeze({
    enabled: true,
    version: VERSION,
    background: LIGHT_BACKGROUND.css,
    stagedAssetPrefix: STAGED_ASSET_PREFIX,
    getSceneId: activeSceneId,
    rewritePackageRootUrl,
  });
})();
